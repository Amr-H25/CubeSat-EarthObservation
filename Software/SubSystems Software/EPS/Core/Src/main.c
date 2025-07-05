/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body - EPS Control System
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
#include "lm35.h"
#include "eps_analog.h"
#include "config.h"
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
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c2;
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
eps_data_t eps_data;
uint32_t last_read_time = 0;
uint32_t last_led_toggle = 0;
uint8_t led_state = 0;
char uart_buffer[200];
char uart_rx_buffer[50];
uint8_t uart_rx_index = 0;
uint8_t uart_rx_char;
volatile uint8_t uart_rx_complete = 0;  // Made volatile for interrupt safety
uint8_t i2c_data_buffer[32];

// Power control states (0 = OFF, 1 = ON)
uint8_t power_states[3] = {0, 0, 0}; // ADCS, COMM, PAYLOAD - start with all OFF
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void Process_UART_Command(void);
void Send_I2C_Data(void);
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
  MX_ADC1_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();

  /* USER CODE BEGIN 2 */
  // Initialize EPS systems
  LM35_Init(&hadc1);
  EPS_Analog_Init(&hadc1);

  // Initialize power control (all subsystems OFF initially)
  EPS_PowerControlAll(0);
  power_states[0] = 0; // ADCS OFF
  power_states[1] = 0; // COMM OFF
  power_states[2] = 0; // PAYLOAD OFF

  // Initialize LED (PC13) OFF
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  // Send startup messages
  HAL_Delay(100); // Small delay to ensure UART is ready
  strcpy(uart_buffer, "\r\n=== EPS System Initialized ===\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  strcpy(uart_buffer, "All subsystems powered OFF by default\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  strcpy(uart_buffer, "Available commands:\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  strcpy(uart_buffer, "  ADCS_ON, ADCS_OFF\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  strcpy(uart_buffer, "  COMM_ON, COMM_OFF\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  strcpy(uart_buffer, "  PAYLOAD_ON, PAYLOAD_OFF\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  strcpy(uart_buffer, "  STATUS\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  strcpy(uart_buffer, "Ready for commands...\r\n\r\n");
  HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

  // Initialize UART receive buffer and start interrupt-based reception
  memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
  uart_rx_index = 0;
  uart_rx_complete = 0;

  // Start UART reception interrupt
  if (HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1) != HAL_OK) {
      strcpy(uart_buffer, "ERROR: Failed to start UART reception\r\n");
      HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
  }

  last_read_time = HAL_GetTick();
  last_led_toggle = HAL_GetTick();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    uint32_t current_time = HAL_GetTick();

    // Toggle LED every 500ms to indicate system is alive
    if (current_time - last_led_toggle >= 500) {
        last_led_toggle = current_time;
        led_state = !led_state;
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, led_state ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }

    // Check for completed UART commands
    if (uart_rx_complete) {
        uart_rx_complete = 0;
        Process_UART_Command();
    }

    // Read sensors every EPS_READ_INTERVAL_MS (1 second)
    if (current_time - last_read_time >= EPS_READ_INTERVAL_MS) {
        last_read_time = current_time;

        // Read all analog channels
        if (EPS_ReadAllChannels(&hadc1, &eps_data) == HAL_OK) {

            // Format and send detailed readings via UART
            strcpy(uart_buffer, "=== EPS READINGS ===\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

            snprintf(uart_buffer, sizeof(uart_buffer), "Temperature: %.2f °C\r\n", eps_data.temperature);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

            snprintf(uart_buffer, sizeof(uart_buffer), "Solar Panel +X: %.3f V\r\n", eps_data.solar_p_x);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

            snprintf(uart_buffer, sizeof(uart_buffer), "Solar Panel -X: %.3f V\r\n", eps_data.solar_n_x);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

            snprintf(uart_buffer, sizeof(uart_buffer), "Solar Panel +Y: %.3f V\r\n", eps_data.solar_p_y);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

            snprintf(uart_buffer, sizeof(uart_buffer), "Solar Panel -Y: %.3f V\r\n", eps_data.solar_n_y);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

            snprintf(uart_buffer, sizeof(uart_buffer), "Battery Voltage: %.3f V\r\n", eps_data.battery_voltage);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

            snprintf(uart_buffer, sizeof(uart_buffer), "Current Sensor: %.3f V\r\n", eps_data.current_sensor);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

            // Calculate and display total solar power
            float solar_total = eps_data.solar_p_x + eps_data.solar_n_x + eps_data.solar_p_y + eps_data.solar_n_y;
            snprintf(uart_buffer, sizeof(uart_buffer), "Total Solar Voltage: %.3f V\r\n", solar_total);
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

            // Display current power status
            snprintf(uart_buffer, sizeof(uart_buffer), "Power Status: ADCS=%s, COMM=%s, PAYLOAD=%s\r\n",
                     power_states[0] ? "ON" : "OFF",
                     power_states[1] ? "ON" : "OFF",
                     power_states[2] ? "ON" : "OFF");
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

            strcpy(uart_buffer, "==================\r\n\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

            // Send data via I2C to other subsystems
            Send_I2C_Data();

        } else {
            strcpy(uart_buffer, "ERROR: Failed to read analog channels\r\n");
            HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
        }
    }

    HAL_Delay(10);  // Small delay to prevent overwhelming the system
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

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{
  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{
  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */
}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
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
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB13 PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief  UART Rx Transfer completed callback
  * @param  huart: UART handle
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        if (uart_rx_char == '\r' || uart_rx_char == '\n')
        {
            // Command complete - only process if we have received characters
            if (uart_rx_index > 0)
            {
                uart_rx_buffer[uart_rx_index] = '\0';
                uart_rx_complete = 1;
                uart_rx_index = 0;
            }
        }
        else if (uart_rx_index < sizeof(uart_rx_buffer) - 1)
        {
            // Store character and increment index
            uart_rx_buffer[uart_rx_index++] = uart_rx_char;
        }
        else
        {
            // Buffer overflow - reset
            uart_rx_index = 0;
            memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
        }

        // Restart reception for next character
        HAL_UART_Receive_IT(&huart1, &uart_rx_char, 1);
    }
}

/**
  * @brief  Process received UART command
  * @param  None
  * @retval None
  */
void Process_UART_Command(void)
{
    // Convert to uppercase for case-insensitive comparison
    for (int i = 0; uart_rx_buffer[i]; i++) {
        if (uart_rx_buffer[i] >= 'a' && uart_rx_buffer[i] <= 'z') {
            uart_rx_buffer[i] = uart_rx_buffer[i] - 'a' + 'A';
        }
    }

    // Echo received command for debugging
    snprintf(uart_buffer, sizeof(uart_buffer), "Received: %s\r\n", uart_rx_buffer);
    HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);

    // Process commands
    if (strcmp(uart_rx_buffer, "ADCS_OFF") == 0) {
        EPS_PowerControl(POWER_SUBSYS_1, 0);
        power_states[0] = 0;
        strcpy(uart_buffer, "COMMAND: ADCS subsystem powered OFF (PB12=LOW)\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    }
    else if (strcmp(uart_rx_buffer, "ADCS_ON") == 0) {
        EPS_PowerControl(POWER_SUBSYS_1, 1);
        power_states[0] = 1;
        strcpy(uart_buffer, "COMMAND: ADCS subsystem powered ON (PB12=HIGH)\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    }
    else if (strcmp(uart_rx_buffer, "COMM_OFF") == 0) {
        EPS_PowerControl(POWER_SUBSYS_2, 0);
        power_states[1] = 0;
        strcpy(uart_buffer, "COMMAND: COMM subsystem powered OFF (PB13=LOW)\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    }
    else if (strcmp(uart_rx_buffer, "COMM_ON") == 0) {
        EPS_PowerControl(POWER_SUBSYS_2, 1);
        power_states[1] = 1;
        strcpy(uart_buffer, "COMMAND: COMM subsystem powered ON (PB13=HIGH)\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    }
    else if (strcmp(uart_rx_buffer, "PAYLOAD_OFF") == 0) {
        EPS_PowerControl(POWER_SUBSYS_3, 0);
        power_states[2] = 0;
        strcpy(uart_buffer, "COMMAND: PAYLOAD subsystem powered OFF (PB14=LOW)\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    }
    else if (strcmp(uart_rx_buffer, "PAYLOAD_ON") == 0) {
        EPS_PowerControl(POWER_SUBSYS_3, 1);
        power_states[2] = 1;
        strcpy(uart_buffer, "COMMAND: PAYLOAD subsystem powered ON (PB14=HIGH)\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    }
    else if (strcmp(uart_rx_buffer, "STATUS") == 0) {
        snprintf(uart_buffer, sizeof(uart_buffer), "POWER STATUS:\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
        snprintf(uart_buffer, sizeof(uart_buffer), "  ADCS (PB12): %s\r\n", power_states[0] ? "ON" : "OFF");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
        snprintf(uart_buffer, sizeof(uart_buffer), "  COMM (PB13): %s\r\n", power_states[1] ? "ON" : "OFF");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
        snprintf(uart_buffer, sizeof(uart_buffer), "  PAYLOAD (PB14): %s\r\n", power_states[2] ? "ON" : "OFF");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    }
    else {
        snprintf(uart_buffer, sizeof(uart_buffer), "ERROR: Unknown command '%s'\r\n", uart_rx_buffer);
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
        strcpy(uart_buffer, "Valid commands: ADCS_ON, ADCS_OFF, COMM_ON, COMM_OFF, PAYLOAD_ON, PAYLOAD_OFF, STATUS\r\n");
        HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    }

    // Clear the receive buffer
    memset(uart_rx_buffer, 0, sizeof(uart_rx_buffer));
}

/**
  * @brief  Send EPS data via I2C to other subsystems
  * @param  None
  * @retval None
  */
void Send_I2C_Data(void)
{
    // Prepare I2C data buffer with EPS readings
    float solar_total = eps_data.solar_p_x + eps_data.solar_n_x + eps_data.solar_p_y + eps_data.solar_n_y;

    // Pack data into I2C buffer (integer and fractional parts)
    i2c_data_buffer[0] = (uint8_t)eps_data.temperature;
    i2c_data_buffer[1] = (uint8_t)((eps_data.temperature - (uint8_t)eps_data.temperature) * 100);
    i2c_data_buffer[2] = (uint8_t)eps_data.battery_voltage;
    i2c_data_buffer[3] = (uint8_t)((eps_data.battery_voltage - (uint8_t)eps_data.battery_voltage) * 100);
    i2c_data_buffer[4] = (uint8_t)solar_total;
    i2c_data_buffer[5] = (uint8_t)((solar_total - (uint8_t)solar_total) * 100);
    i2c_data_buffer[6] = (uint8_t)eps_data.current_sensor;
    i2c_data_buffer[7] = (uint8_t)((eps_data.current_sensor - (uint8_t)eps_data.current_sensor) * 100);
    i2c_data_buffer[8] = power_states[0]; // ADCS power state
    i2c_data_buffer[9] = power_states[1]; // COMM power state
    i2c_data_buffer[10] = power_states[2]; // PAYLOAD power state

    // Try to send data via I2C (this will fail if no device is connected, which is okay)
    // You can implement specific I2C addresses for different subsystems as needed
    HAL_StatusTypeDef result = HAL_I2C_Master_Transmit(&hi2c2, EPS_I2C_ADDRESS << 1,
                                                       i2c_data_buffer, 11, EPS_I2C_TIMEOUT_MS);

    // Optional: Log I2C transmission status (comment out to reduce UART spam)
    // if (result != HAL_OK) {
    //     strcpy(uart_buffer, "I2C: No device connected (this is normal during testing)\r\n");
    //     HAL_UART_Transmit(&huart1, (uint8_t*)uart_buffer, strlen(uart_buffer), 1000);
    // }
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
