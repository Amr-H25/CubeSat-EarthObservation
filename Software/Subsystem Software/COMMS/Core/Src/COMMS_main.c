/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - COMMS Subsystem
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdio.h>  // Added for printf and snprintf

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hc05_driver.h"
#include <string.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
// Packet structure matching ADCS sender
typedef struct {
    uint8_t start_marker;
    uint8_t packet_id;
    uint16_t data_length;
    float temperature;
    struct {
        uint16_t front;
        uint16_t right;
        uint16_t back;
        uint16_t left;
    } ldr_values;
    struct {
        float latitude;
        float longitude;
        float altitude;
    } gps_position;
    struct {
        float x;
        float y;
        float z;
    } accel;
    struct {
        float x;
        float y;
        float z;
    } gyro;
    float mpu_temp;
    uint8_t checksum;
    uint8_t end_marker;
} __attribute__((packed)) SensorDataPacket_t;

// Processed sensor data structure
typedef struct {
    float temperature;
    struct {
        uint16_t front;
        uint16_t right;
        uint16_t back;
        uint16_t left;
    } ldr_values;
    struct {
        float latitude;
        float longitude;
        float altitude;
    } gps_position;
    struct {
        float x;
        float y;
        float z;
    } accel;
    struct {
        float x;
        float y;
        float z;
    } gyro;
    float mpu_temp;
} SensorData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define ADCS_UART &huart1  // UART1 for ADCS communication
#define DEBUG_UART &huart2 // UART2 for debug printing
#define COMMS_UART &huart3 // UART3 for Bluetooth communication
#define SENSOR_PACKET_SIZE sizeof(SensorDataPacket_t)
#define PACKET_START_MARKER 0xAA
#define PACKET_END_MARKER 0x55
#define SENSOR_DATA_PACKET_ID 0x01
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
// Global telemetry data structure
TelemetryData telemetry = {
    .timestamp = "2025-07-03 12:00:00",
    .system_status = "ONLINE",
    .adcs = { .status = "OFFLINE" /* Initial status */ },
    .eps = { .status = "OFFLINE" },
    .obc = { .status = "ONLINE" },
    .comms = { .status = "OFFLINE" },
    .payload = { .status = "OFFLINE" },
    .last_command_button = "NONE",
    .last_command_time = "N/A",
    .command_status = "Idle"
};

// UART receiving variables
volatile bool new_sensor_data_ready = false;
SensorData_t received_sensor_data;
uint8_t sensor_rx_buffer[SENSOR_PACKET_SIZE];
uint16_t sensor_rx_idx = 0;

// State machine for receiving packets
typedef enum {
    WAIT_START,
    WAIT_DATA,
} PacketState_t;
PacketState_t packet_state = WAIT_START;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void ADCS_UART_ReceiveHandler(void);
bool ValidatePacket(SensorDataPacket_t *packet);
void ProcessSensorPacket(SensorDataPacket_t *packet);
void UpdateTelemetryFromSensors(SensorData_t *sensor_data);
uint8_t CalculateChecksum(uint8_t *data, uint16_t length);
void ProcessReceivedCommand(const char* command);
void ExecuteCommand(const char* command);
void UpdateSystemStatus(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
// Redirect printf to DEBUG_UART (USART2)
int _write(int file, char *ptr, int len) {
    HAL_UART_Transmit(DEBUG_UART, (uint8_t*)ptr, len, HAL_MAX_DELAY);
    return len;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* MCU Configuration--------------------------------------------------------*/
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  if (BT_Init(COMMS_UART) != BT_OK) {
    Error_Handler();
  }
  printf("COMMS System Initialized\r\n");
  UpdateSystemStatus();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // 1. Continuously check for and process incoming data from ADCS
    ADCS_UART_ReceiveHandler();

    // 2. If a full, valid packet has been received, update telemetry
    if(new_sensor_data_ready) {
        new_sensor_data_ready = false; // Reset flag
        UpdateTelemetryFromSensors(&received_sensor_data);
        printf("ADCS Telemetry Updated.\r\n");
    }

    // 3. Check for commands received from ground station via Bluetooth
    if(BT_IsCommandReceived()) {
        char cmd[COMMAND_BUFFER_SIZE];
        if(BT_ReceiveCommand(cmd, 10) == BT_OK) {
            ProcessReceivedCommand(cmd);
        }
        BT_ClearCommandFlag();
    }

    // 4. Periodically send the full telemetry packet via Bluetooth
    static uint32_t last_telemetry_tx = 0;
    if(HAL_GetTick() - last_telemetry_tx >= 10000) { // Send every 10 seconds
        last_telemetry_tx = HAL_GetTick();
        UpdateSystemStatus(); // Update statuses before sending
        if(BT_SendJSON(&telemetry) == BT_OK) {
            telemetry.comms.packets_sent++;
            printf("Telemetry JSON sent via Bluetooth.\r\n");
        }
    }

    // 5. Blink LED as a heartbeat
    static uint32_t last_heartbeat = 0;
    if(HAL_GetTick() - last_heartbeat >= 500) {
        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
        last_heartbeat = HAL_GetTick();
    }

    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/* ... (System Clock and Peripheral Initialization functions remain the same) ... */
/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PA1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief Handles receiving bytes from ADCS UART and reconstructing packets.
  * This function should be called repeatedly in the main loop.
  */
void ADCS_UART_ReceiveHandler(void) {
    uint8_t byte;
    // Check for a new byte with a very short timeout (non-blocking)
    if (HAL_UART_Receive(ADCS_UART, &byte, 1, 0) == HAL_OK) {
        switch (packet_state) {
            case WAIT_START:
                if (byte == PACKET_START_MARKER) {
                    sensor_rx_idx = 0;
                    sensor_rx_buffer[sensor_rx_idx++] = byte;
                    packet_state = WAIT_DATA;
                }
                break;

            case WAIT_DATA:
                if (sensor_rx_idx < SENSOR_PACKET_SIZE) {
                    sensor_rx_buffer[sensor_rx_idx++] = byte;
                }
                // If the buffer is now full, process the packet
                if (sensor_rx_idx >= SENSOR_PACKET_SIZE) {
                    ProcessSensorPacket((SensorDataPacket_t*)sensor_rx_buffer);
                    packet_state = WAIT_START; // Reset for next packet
                }
                break;
        }
    }
}

/**
  * @brief Calculates a simple XOR checksum. Must match the sender's implementation.
  */
uint8_t CalculateChecksum(uint8_t *data, uint16_t length) {
    uint8_t checksum = 0;
    for (uint16_t i = 0; i < length; i++) {
        checksum ^= data[i];
    }
    return checksum;
}

/**
  * @brief Validates a received sensor packet.
  * @retval true if packet is valid, false otherwise.
  */
bool ValidatePacket(SensorDataPacket_t *packet) {
    // 1. Check start and end markers
    if (packet->start_marker != PACKET_START_MARKER || packet->end_marker != PACKET_END_MARKER) {
        printf("Error: Invalid markers\r\n");
        return false;
    }
    // 2. Check packet ID
    if (packet->packet_id != SENSOR_DATA_PACKET_ID) {
        printf("Error: Invalid packet ID\r\n");
        return false;
    }
    // 3. Validate checksum
    uint16_t checksum_len = sizeof(SensorDataPacket_t) - 4; // Length from packet_id to mpu_temp
    uint8_t calculated_checksum = CalculateChecksum((uint8_t*)&packet->packet_id, checksum_len);
    if (calculated_checksum != packet->checksum) {
        printf("Error: Checksum mismatch. Got: %u, Expected: %u\r\n", packet->checksum, calculated_checksum);
        return false;
    }
    return true;
}

/**
  * @brief Processes a complete sensor packet after it's received.
  */
/**
  * @brief Processes a complete sensor packet after it's received.
  */
void ProcessSensorPacket(SensorDataPacket_t *packet) {
    if (!ValidatePacket(packet)) {
        printf("Invalid sensor packet received.\r\n");
        return;
    }

    // Debug print received values
    printf("[COMMS] Received sensor data:\r\n");
    printf("  Temp: %.2f C\r\n", packet->temperature);
    printf("  LDR: F=%d, R=%d, B=%d, L=%d\r\n",
           packet->ldr_values.front, packet->ldr_values.right,
           packet->ldr_values.back, packet->ldr_values.left);
    printf("  GPS: Lat=%.4f, Lon=%.4f, Alt=%.1f\r\n",
           packet->gps_position.latitude, packet->gps_position.longitude,
           packet->gps_position.altitude);
    printf("  Accel: X=%.2f, Y=%.2f, Z=%.2f\r\n",
           packet->accel.x, packet->accel.y, packet->accel.z);
    printf("  Gyro: X=%.2f, Y=%.2f, Z=%.2f\r\n",
           packet->gyro.x, packet->gyro.y, packet->gyro.z);
    printf("  MPU Temp: %.2f C\r\n", packet->mpu_temp);

    // Copy data to global structure
    received_sensor_data.temperature = packet->temperature;
    memcpy(&received_sensor_data.ldr_values, &packet->ldr_values, sizeof(packet->ldr_values));
    memcpy(&received_sensor_data.gps_position, &packet->gps_position, sizeof(packet->gps_position));
    memcpy(&received_sensor_data.accel, &packet->accel, sizeof(packet->accel));
    memcpy(&received_sensor_data.gyro, &packet->gyro, sizeof(packet->gyro));
    received_sensor_data.mpu_temp = packet->mpu_temp;

    new_sensor_data_ready = true;
}


/**
  * @brief Updates the main telemetry structure with fresh data from sensors.
  */
void UpdateTelemetryFromSensors(SensorData_t *sensor_data) {
    // Update ADCS status to ONLINE since we're receiving data
    strcpy(telemetry.adcs.status, "ONLINE");

    // Map direct readings
    telemetry.adcs.temperature = (int)sensor_data->temperature;
    telemetry.adcs.light_sensors.face_a = sensor_data->ldr_values.front;
    telemetry.adcs.light_sensors.face_b = sensor_data->ldr_values.right;
    telemetry.adcs.light_sensors.face_c = sensor_data->ldr_values.back;
    telemetry.adcs.light_sensors.face_d = sensor_data->ldr_values.left;

    // Copy GPS data
    memcpy(&telemetry.adcs.gps, &sensor_data->gps_position, sizeof(telemetry.adcs.gps));

    // Calculate orientation from accelerometer
    telemetry.adcs.roll = atan2f(sensor_data->accel.y, sensor_data->accel.z) * (180.0f / M_PI);
    telemetry.adcs.pitch = atan2f(-sensor_data->accel.x,
                                 sqrtf(sensor_data->accel.y * sensor_data->accel.y +
                                       sensor_data->accel.z * sensor_data->accel.z)) * (180.0f / M_PI);
    telemetry.adcs.yaw = sensor_data->gyro.z; // Using gyro Z as yaw rate

    printf("[COMMS] Updated telemetry with new sensor data\r\n");
}

/**
  * @brief Processes commands received over Bluetooth.
  */
void ProcessReceivedCommand(const char* command) {
    printf("Received command: %s\r\n", command);
    strncpy(telemetry.last_command_button, command, sizeof(telemetry.last_command_button) - 1);

    // Update command timestamp
    uint32_t secs = HAL_GetTick() / 1000;
    snprintf(telemetry.last_command_time, sizeof(telemetry.last_command_time), "T+%lu s", secs);

    ExecuteCommand(command);
}

/**
  * @brief Executes a specific action based on the command string.
  */
void ExecuteCommand(const char* command) {
    strcpy(telemetry.command_status, "Executing");

    if (strcmp(command, "STATUS_PING") == 0) {
        printf("Executing STATUS_PING\r\n");
        UpdateSystemStatus();
        strcpy(telemetry.command_status, "Ping Acknowledged");
    } else if (strcmp(command, "REBOOT_SYS") == 0) {
        printf("Executing REBOOT_SYS\r\n");
        strcpy(telemetry.command_status, "Rebooting...");
        HAL_Delay(1000);
        HAL_NVIC_SystemReset();
    } else {
        printf("Unknown command: %s\r\n", command);
        strcpy(telemetry.command_status, "Unknown Command");
    }
}

/**
  * @brief Updates the overall system status based on subsystem health.
  */
void UpdateSystemStatus(void) {
    // Update COMMS status
    strcpy(telemetry.comms.status, "ONLINE");

    // Update overall system status
    if (strcmp(telemetry.adcs.status, "ONLINE") == 0 && strcmp(telemetry.obc.status, "ONLINE") == 0) {
        strcpy(telemetry.system_status, "NOMINAL");
    } else {
        strcpy(telemetry.system_status, "DEGRADED");
    }

    // Update timestamp
    uint32_t secs = HAL_GetTick() / 1000;
    snprintf(telemetry.timestamp, sizeof(telemetry.timestamp), "T+%lu s", secs);
}

/* ... (TIM2_Init, Error_Handler, etc. remain the same) ... */
static void MX_TIM2_Init(void)
{
  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 0xFFFFFFFF;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
    printf("--- SYSTEM ERROR ---\r\n");
  __disable_irq();
  while (1)
  {
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
      HAL_Delay(100);
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  * where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
