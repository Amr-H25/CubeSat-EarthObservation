/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "hc05_driver.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

/* USER CODE BEGIN PV */
// Global telemetry data structure
TelemetryData telemetry = {
    .timestamp = "2025-07-03 12:00:00",
    .system_status = "ONLINE",
    .adcs = {
        .status = "ONLINE",
        .pitch = 12.3f,
        .roll = -5.7f,
        .yaw = 80.0f,
        .gps = {
            .latitude = 31.2001f,
            .longitude = 29.9187f,
            .altitude = 407.2f
        },
        .light_sensors = {
            .face_a = 842,
            .face_b = 156,
            .face_c = 102,
            .face_d = 390
        },
        .temperature = 80
    },
    .eps = {
        .status = "OFFLINE",
        .battery_voltage = 7.4f,
        .battery_percentage = 22,
        .current = 0.32f,
        .solar_power = 2.1f,
        .charging_status = "NOT Charging",
        .boost_output = 12,
        .temperature = 50
    },
    .obc = {
        .status = "ONLINE",
        .data_logging = "NOT ACTIVE",
        .storage_usage = 86,
        .rtc_sync = "OK",
        .i2c_status = "NOT Connected",
        .command_queue = 3
    },
    .comms = {
        .status = "OFFLINE",
        .rf_link = "MEDIUM",
        .signal_strength = -85,
        .packets_sent = 1000,
        .packets_received = 1198,
        .last_command = "RESET"
    },
    .payload = {
        .status = "OFFLINE",
        .payload_status = "CAPTURING",
        .images_today = 47,
        .last_image_size = 2.3f,
        .ai_classification = {
            .result = "Ocean Surface",
            .confidence = 94.2f
        }
    },
    .last_command_button = "NONE",
    .last_command_time = "2025-07-03T12:00:00.000Z",
    .command_status = "Idle"
};
char received_command[COMMAND_BUFFER_SIZE];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */
void ProcessReceivedCommand(const char* command);
void UpdateSystemStatus(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_TIM2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  /* Initialize HC-05 Bluetooth module */
  if (BT_Init(&huart3) != BT_OK) {
    Error_Handler();
  }

  // Start timer for periodic data transmission
  HAL_TIM_Base_Start(&htim2);

  // Initial system status update
  UpdateSystemStatus();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   // Heartbeat with proper static variable handling
	    static uint32_t last_heartbeat = 0;
	    if(HAL_GetTick() - last_heartbeat >= 1000) {
	        HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
	        last_heartbeat = HAL_GetTick();
	        printf("[SYS] Heartbeat %lu\r\n", HAL_GetTick()/1000);
	    }

	    // Handle LED blinking for command reception feedback
	    if(BT_ShouldBlinkLED()) {
	        // Quick blink to indicate command received
	        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	        HAL_Delay(50);
	        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
	        HAL_Delay(50);
	        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
	    }

	    // Command processing with better error handling
	    if(BT_IsCommandReceived()) {
	        char cmd[COMMAND_BUFFER_SIZE];
	        BT_StatusTypeDef status = BT_ReceiveCommand(cmd, 10); // Short timeout

	        if(status == BT_OK) {
	            printf("[CMD] Processing: %s\r\n", cmd);
	            ProcessReceivedCommand(cmd);
	        } else if(status == BT_TIMEOUT) {
	            printf("[CMD] Timeout waiting for command\r\n");
	        }

	        BT_ClearCommandFlag();
	    }

	    // Telemetry transmission
	    static uint32_t last_telemetry = 0;
	    if(HAL_GetTick() - last_telemetry >= 10000) {
	        printf("[SYS] Sending telemetry...\r\n");
	        UpdateSystemStatus();

	        BT_StatusTypeDef status = BT_SendJSON(&telemetry);
	        if(status == BT_OK) {
	            printf("[SYS] Telemetry sent successfully\r\n");
	        } else if(status == BT_BUSY) {
	            printf("[WARN] UART busy, skipping this telemetry cycle\r\n");
	        } else {
	            printf("[ERR] Telemetry send failed! Status: %d\r\n", status);
	        }
	        last_telemetry = HAL_GetTick();
	    }
	    HAL_Delay(10);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

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
}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 7999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 65535;
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
void ProcessReceivedCommand(const char* command) {
    // Simple JSON parser for our specific command
    const char* pattern = "\"last_command_button\":\"";
    char* start = strstr(command, pattern);

    if (start) {
        start += strlen(pattern); // Move to value
        char* end = strchr(start, '"');
        if (end && (end - start) < sizeof(telemetry.last_command_button)) {
            // Copy the command value
            strncpy(telemetry.last_command_button, start, end - start);
            telemetry.last_command_button[end - start] = '\0';

            // Update status
            strcpy(telemetry.command_status, "CMD_RECEIVED");

            // Visual feedback (LED already toggled in callback)
        }
    }
    printf("Received: %s\n", command); // Add to ProcessReceivedCommand
}

void UpdateSystemStatus(void) {
  // Here you would update your telemetry data from sensors
  // For now, we'll just update the timestamp

  // Update timestamp (example format)
  uint32_t tick = HAL_GetTick();
  uint32_t seconds = tick / 1000;
  uint32_t minutes = seconds / 60;
  uint32_t hours = minutes / 60;
  snprintf(telemetry.timestamp, sizeof(telemetry.timestamp),
           "2025-07-03 %02lu:%02lu:%02lu",
           hours % 24, minutes % 60, seconds % 60);

  // Update last command time
  snprintf(telemetry.last_command_time, sizeof(telemetry.last_command_time),
           "2025-07-03T%02lu:%02lu:%02lu.000Z",
           hours % 24, minutes % 60, seconds % 60);

  // You would add your actual sensor readings here
  // Example:
  // telemetry.adcs.temperature = ReadTemperatureSensor();
  // telemetry.eps.battery_percentage = ReadBatteryLevel();
}
// Redirect printf to USART2 (for debug)
int _write(int file, char *ptr, int len) {
    // Add carriage return before newline
    for(int i=0; i<len; i++) {
        if(ptr[i] == '\n') {
            char cr = '\r';
            HAL_UART_Transmit(&huart2, (uint8_t*)&cr, 1, HAL_MAX_DELAY);
        }
        HAL_UART_Transmit(&huart2, (uint8_t*)&ptr[i], 1, HAL_MAX_DELAY);
    }
    return len;
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
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
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
