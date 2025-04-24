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
#include "gps.h"
#include "mpu6050.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define GPS_TEST_INTERVAL 1000  // Test interval in ms
#define MPU6050_TEST_INTERVAL 500  // Test interval in ms for MPU6050
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
char debug_buffer[256];    // Debug message buffer
GPS_Handle gps_handle;
MPU6050_Handle mpu6050_handle;
uint32_t last_gps_test_time = 0;
uint32_t last_mpu_test_time = 0;

// Motor control parameters (will be replaced with PID controller later)
uint16_t motor_x_pwm = 0;
uint16_t motor_y_pwm = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void PrintDebugMessage(const char* message);
void TestGPSData(void);
void TestMPU6050Data(void);
void UpdateMotorControl(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Print debug message via UART1
 * @param message String message to print
 */
void PrintDebugMessage(const char* message) {
    HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 100);
}

/**
 * @brief Test GPS data by retrieving and displaying position, time, and coordinates
 */
void TestGPSData(void) {
    GPS_Position position;
    GPS_Time time;
    ECEF_Position ecef;
    ECI_Position eci;

    // Print separator for readability
    PrintDebugMessage("\r\n--------- GPS TEST ---------\r\n");

    // Check if GPS has fix
    if (GPS_HasFix(&gps_handle)) {
        PrintDebugMessage("GPS Fix: VALID\r\n");

        // Get and display position
        if (GPS_GetPosition(&gps_handle, &position) == GPS_OK) {
            sprintf(debug_buffer, "Latitude: %.6f\r\nLongitude: %.6f\r\nAltitude: %.2f m\r\n",
                    position.latitude, position.longitude, position.altitude);
            PrintDebugMessage(debug_buffer);

            sprintf(debug_buffer, "Satellites: %d\r\nHDOP: %.2f\r\n",
                    position.satellites, position.hdop);
            PrintDebugMessage(debug_buffer);

            sprintf(debug_buffer, "Speed: %.2f knots\r\nCourse: %.2f degrees\r\n",
                    position.speed, position.course);
            PrintDebugMessage(debug_buffer);
        }

        // Get and display time
        if (GPS_GetTime(&gps_handle, &time) == GPS_OK) {
            sprintf(debug_buffer, "UTC Time: %02d/%02d/%04d %02d:%02d:%02d.%03d\r\n",
                    time.day, time.month, time.year,
                    time.hour, time.minute, time.second, time.millisecond);
            PrintDebugMessage(debug_buffer);
        }

        // Get and display ECEF coordinates
        if (GPS_GetECEFPosition(&gps_handle, &ecef) == GPS_OK) {
            sprintf(debug_buffer, "ECEF Position:\r\nX: %.2f m\r\nY: %.2f m\r\nZ: %.2f m\r\n",
                    ecef.x, ecef.y, ecef.z);
            PrintDebugMessage(debug_buffer);
        }

        // Get and display ECI coordinates
        if (GPS_GetECIPosition(&gps_handle, &eci) == GPS_OK) {
            sprintf(debug_buffer, "ECI Position:\r\nX: %.2f m\r\nY: %.2f m\r\nZ: %.2f m\r\n",
                    eci.x, eci.y, eci.z);
            PrintDebugMessage(debug_buffer);

            sprintf(debug_buffer, "ECI Velocity:\r\nVx: %.2f m/s\r\nVy: %.2f m/s\r\nVz: %.2f m/s\r\n",
                    eci.vx, eci.vy, eci.vz);
            PrintDebugMessage(debug_buffer);
        }
    } else {
        PrintDebugMessage("GPS Fix: NO FIX\r\n");
    }

    PrintDebugMessage("-----------------------------\r\n");
}

/**
 * @brief Test MPU6050 data by retrieving and displaying accelerometer and gyroscope readings
 */
void TestMPU6050Data(void) {
    // Read all data from MPU6050
    if (MPU6050_ReadAllData(&mpu6050_handle) == MPU6050_OK) {
        // Print data through debug UART
        MPU6050_Print(&mpu6050_handle, &huart1);

        // Call motor control update function
        UpdateMotorControl();
    } else {
        PrintDebugMessage("Error reading MPU6050 data\r\n");
    }
}

/**
 * @brief Update motor control based on MPU6050 readings
 * This is a simple placeholder for the future PID controller
 */
void UpdateMotorControl(void) {
    // Simple proportional control for demonstration purposes
    // Will be replaced with full PID controller later

    // Use gyro data for motor control (more responsive for orientation control)
    // X-axis control
    float x_position = mpu6050_handle.scaledGyro.x;

    // Map gyro readings to PWM values (0-1000)
    // Note: This is very basic and will be replaced with PID controller
    if (x_position > 5.0f) {
        // Tilting right - reduce right motor speed
        motor_x_pwm = 500 - (uint16_t)((x_position - 5.0f) * 20.0f);
        if (motor_x_pwm < 0) motor_x_pwm = 0;
    } else if (x_position < -5.0f) {
        // Tilting left - increase right motor speed
        motor_x_pwm = 500 + (uint16_t)((-x_position - 5.0f) * 20.0f);
        if (motor_x_pwm > 999) motor_x_pwm = 999;
    } else {
        // Near level - maintain moderate speed
        motor_x_pwm = 500;
    }

    // Y-axis control
    float y_position = mpu6050_handle.scaledGyro.y;

    // Map gyro readings to PWM values (0-1000)
    if (y_position > 5.0f) {
        // Tilting forward - reduce forward motor speed
        motor_y_pwm = 500 - (uint16_t)((y_position - 5.0f) * 20.0f);
        if (motor_y_pwm < 0) motor_y_pwm = 0;
    } else if (y_position < -5.0f) {
        // Tilting backward - increase forward motor speed
        motor_y_pwm = 500 + (uint16_t)((-y_position - 5.0f) * 20.0f);
        if (motor_y_pwm > 999) motor_y_pwm = 999;
    } else {
        // Near level - maintain moderate speed
        motor_y_pwm = 500;
    }

    // Update PWM channels for motors
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, motor_x_pwm);
    __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, motor_y_pwm);

    // Print motor control values
    sprintf(debug_buffer, "Motor PWM - X: %d, Y: %d\r\n", motor_x_pwm, motor_y_pwm);
    PrintDebugMessage(debug_buffer);
}

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
  MX_USART1_UART_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  // Initialize GPS with UART3 for GPS communication
  PrintDebugMessage("\r\n\r\nCubeSat System Starting...\r\n");

  // Initialize GPS
  PrintDebugMessage("Initializing GPS module...\r\n");
  GPS_Status gps_status = GPS_Init(&gps_handle, &huart3);
  if (gps_status == GPS_OK) {
      PrintDebugMessage("GPS Initialized Successfully\r\n");
  } else {
      PrintDebugMessage("GPS Initialization Failed\r\n");
  }

  // Initialize MPU6050
  PrintDebugMessage("Initializing MPU6050 module...\r\n");
  MPU6050_Status mpu_status = MPU6050_Init(&mpu6050_handle, &hi2c1);
  if (mpu_status == MPU6050_OK) {
      PrintDebugMessage("MPU6050 Initialized Successfully\r\n");

      // Calibrate MPU6050
      PrintDebugMessage("Calibrating MPU6050...\r\n");
      if (MPU6050_Calibrate(&mpu6050_handle, 100) == MPU6050_OK) {
          PrintDebugMessage("MPU6050 Calibration Complete\r\n");
      } else {
          PrintDebugMessage("MPU6050 Calibration Failed\r\n");
      }
  } else {
      PrintDebugMessage("MPU6050 Initialization Failed\r\n");
  }

  // Start PWM for motor control
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);  // X-axis motor
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);  // Y-axis motor

  // Set initial motor speeds to mid-range (500 out of 999)
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    // Process GPS data
    GPS_Process(&gps_handle);

    // Periodically test and display GPS data
    uint32_t current_time = HAL_GetTick();
    if (current_time - last_gps_test_time >= GPS_TEST_INTERVAL) {
        last_gps_test_time = current_time;
        TestGPSData();
    }

    // Periodically test and display MPU6050 data
    if (current_time - last_mpu_test_time >= MPU6050_TEST_INTERVAL) {
        last_mpu_test_time = current_time;
        TestMPU6050Data();
    }

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
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

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
  huart1.Init.BaudRate = 9600;
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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1|IN1_Pin|GPIO_PIN_15|IN2_Pin
                          |IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PB1 IN1_Pin PB15 IN2_Pin
                           IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_1|IN1_Pin|GPIO_PIN_15|IN2_Pin
                          |IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
 * @brief UART RX Complete callback
 * @param huart UART handle
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3) {
    // Call GPS driver callback
    GPS_UART_RxCpltCallback(&gps_handle);
  }
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
