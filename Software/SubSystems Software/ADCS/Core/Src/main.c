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
#include "mpu6050.h"
#include "gps.h"
#include "l298n.h"
#include "ldr.h"
#include "lm35.h"
#include <string.h>
#include <stdio.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float temperature;
    LightSensor_Values ldr_values;
    GPS_Position gps_position;
    GPS_Time gps_time;
    MPU6050_ScaledData accel;
    MPU6050_ScaledData gyro;
    float mpu_temp;
} SensorData_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SENSOR_UPDATE_DELAY_MS 1000  // Delay between sensor updates
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;
TIM_HandleTypeDef htim4;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
L298N_HandleTypeDef hmotor;
GPS_Handle hgps;
MPU6050_Handle hmpu;
SensorData_t sensor_data;
MPU6050_ScaledData prev_accel = {0};
MPU6050_ScaledData prev_gyro = {0};

typedef enum {
    DIRECTION_NONE,
    DIRECTION_POSITIVE_X,
    DIRECTION_NEGATIVE_X,
    DIRECTION_POSITIVE_Y,
    DIRECTION_NEGATIVE_Y,
    DIRECTION_POSITIVE_Z,
    DIRECTION_NEGATIVE_Z
} MovementDirection;

MovementDirection last_movement_direction = DIRECTION_NONE;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void UART_Print(const char *message);
void UART_PrintFloat(const char *label, float value);
void UART_PrintLDRs(LightSensor_Values *values);
void UART_PrintGPSData(GPS_Position *position, GPS_Time *time);
void UART_PrintMPUData(MPU6050_ScaledData *accel, MPU6050_ScaledData *gyro, float temp);
void ReadAllSensors(SensorData_t *data);

void SetMPUOrigin(MPU6050_Handle *hmpu);
bool HasMovedFromOrigin(MPU6050_Handle *hmpu);
MovementDirection DetectMovementDirection(MPU6050_Handle *hmpu);
void ActivateMotorReaction(L298N_HandleTypeDef *hmotor, MovementDirection dir);
void TestMotorA(void);
void SimpleMotorControl(MPU6050_Handle *hmpu, L298N_HandleTypeDef *hmotor);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        GPS_UART_RxCpltCallback(&hgps);
    }
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
    MX_ADC1_Init();
    MX_USART1_UART_Init();
    MX_TIM4_Init();
    MX_I2C1_Init();
    MX_I2C2_Init();
    MX_USART2_UART_Init();

    /* USER CODE BEGIN 2 */
    // Initialize all sensors and modules
    LM35_Init(&hadc1);
    LightSensor_Init(&hadc1);

    // Initialize motor driver
    L298N_InitTypeDef motor_init = {
        .htim = &htim4,
        .channel_a = TIM_CHANNEL_1,
        .channel_b = TIM_CHANNEL_2,
        .mode = L298N_SINGLE_CHANNEL
    };
    L298N_Init(&hmotor, &motor_init);

    // Initialize GPS module with UART2
    if (GPS_Init(&hgps, &huart2) != GPS_OK) {
        UART_Print("GPS initialization failed!");
        Error_Handler();
    }

    // Initialize MPU6050
    if (MPU6050_Init(&hmpu, &hi2c1) != MPU6050_OK) {
        UART_Print("MPU6050 initialization failed!");
        Error_Handler();
    }

    // Calibrate MPU6050 gyroscope
    if (MPU6050_Calibrate(&hmpu, 100) != MPU6050_OK) {
        UART_Print("MPU6050 calibration failed!");
        Error_Handler();
    }

    SetMPUOrigin(&hmpu);

    UART_Print("System initialized. Starting sensor readings...");
    HAL_Delay(1000);
    TestMotorA();
    /* USER CODE END 2 */

    /* Infinite loop */
    /* USER CODE BEGIN WHILE */
    while (1) {

        ReadAllSensors(&sensor_data);

        // Display sensor data via UART
        UART_Print("\r\n----- Sensor Readings -----");

        // Print temperature
        UART_PrintFloat("LM35 Temperature", sensor_data.temperature);

        // Print LDR values
        UART_PrintLDRs(&sensor_data.ldr_values);

        // Print GPS data if available
        if (GPS_HasFix(&hgps)) {
            UART_PrintGPSData(&sensor_data.gps_position, &sensor_data.gps_time);
        } else {
            UART_Print("GPS: No fix available");
        }

        // Print MPU6050 data
        UART_PrintMPUData(&sensor_data.accel, &sensor_data.gyro, sensor_data.mpu_temp);

        // Simplified motor control based on MPU movement
        SimpleMotorControl(&hmpu, &hmotor);

        HAL_Delay(SENSOR_UPDATE_DELAY_MS);
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
  huart2.Init.BaudRate = 9600;
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
  HAL_GPIO_WritePin(GPIOB, IN1_Pin|GPIO_PIN_15|IN2_Pin|IN3_Pin
                          |IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : IN1_Pin PB15 IN2_Pin IN3_Pin
                           IN4_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|GPIO_PIN_15|IN2_Pin|IN3_Pin
                          |IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void TestMotorA(void) {
    char buffer[64];

    // Print test start message
    snprintf(buffer, sizeof(buffer), "Starting Motor A test - IN1 HIGH, IN2 LOW for 10 seconds");
    UART_Print(buffer);

    // Set motor speed (adjust as needed, 50% for testing)
    L298N_SetSpeed(&hmotor, MOTOR_A, 50);

    // Set direction: IN1 HIGH, IN2 LOW
    HAL_GPIO_WritePin(hmotor.in1_port, hmotor.in1_pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(hmotor.in2_port, hmotor.in2_pin, GPIO_PIN_RESET);

    // Print pin states
    snprintf(buffer, sizeof(buffer), "IN1: HIGH, IN2: LOW, PWM: 50%%");
    UART_Print(buffer);

    // Run for 10 seconds
    HAL_Delay(10000);

    // Stop motor
    L298N_StopMotor(&hmotor, MOTOR_A);
    UART_Print("Motor A test complete - Motor stopped");
}

/* Modified movement detection and motor control */
void SimpleMotorControl(MPU6050_Handle *hmpu, L298N_HandleTypeDef *hmotor) {
    if (HasMovedFromOrigin(hmpu)) {
        UART_Print("Movement detected - Starting motor");

        // Set motor speed (adjust percentage as needed)
        L298N_SetSpeed(hmotor, MOTOR_A, 70); // 70% speed

        // Set direction (forward in this case)
        L298N_SetDirection(hmotor, MOTOR_A, MOTOR_FORWARD);
    } else {
        UART_Print("At origin - Stopping motor");
        L298N_StopMotor(hmotor, MOTOR_A);
    }
}
void UART_Print(const char *message) {
    HAL_UART_Transmit(&huart1, (uint8_t*)message, strlen(message), 100);
    // Add newline
    HAL_UART_Transmit(&huart1, (uint8_t*)"\r\n", 2, 100);
}

void UART_PrintFloat(const char *label, float value) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "%s: %.2f", label, value);
    UART_Print(buffer);
}

void UART_PrintLDRs(LightSensor_Values *values) {
    char buffer[64];
    snprintf(buffer, sizeof(buffer), "LDRs - Front:%4u Right:%4u Back:%4u Left:%4u",
            values->front, values->right, values->back, values->left);
    UART_Print(buffer);
}

void UART_PrintGPSData(GPS_Position *position, GPS_Time *time) {
    char buffer[128];

    // Print position
    snprintf(buffer, sizeof(buffer), "GPS Position: Lat:%.6f, Lon:%.6f, Alt:%.2fm",
            position->latitude, position->longitude, position->altitude);
    UART_Print(buffer);

    // Print time
    snprintf(buffer, sizeof(buffer), "GPS Time: %02d:%02d:%02d.%03d %02d/%02d/%04d",
            time->hour, time->minute, time->second, time->millisecond,
            time->day, time->month, time->year);
    UART_Print(buffer);

    // Print additional GPS info
    snprintf(buffer, sizeof(buffer), "GPS Info: Speed:%.2f knots, Course:%.2f°, Sats:%d",
            position->speed, position->course, position->satellites);
    UART_Print(buffer);
}


void ReadAllSensors(SensorData_t *data) {
    // Read LM35 temperature
    data->temperature = LM35_ReadTemperature(&hadc1);

    // Read LDR values
    LightSensor_ReadAll(&hadc1, &data->ldr_values);

    // Process GPS data
    GPS_Process(&hgps);
    if (GPS_HasFix(&hgps)) {
        GPS_GetPosition(&hgps, &data->gps_position);
        GPS_GetTime(&hgps, &data->gps_time);
    }

    // Read MPU6050 data
    if (MPU6050_ReadAllData(&hmpu) == MPU6050_OK) {
        data->accel = hmpu.scaledAccel;
        data->gyro = hmpu.scaledGyro;
        data->mpu_temp = hmpu.temperature;
    }
}

void CalculateOrientation(MPU6050_ScaledData *accel, MPU6050_ScaledData *gyro, MPU6050_Orientation *orient) {
    // Calculate pitch and roll from accelerometer (in degrees)
    orient->pitch = atan2f(-accel->x, sqrtf(accel->y * accel->y + accel->z * accel->z)) * 180.0f / M_PI;
    orient->roll = atan2f(accel->y, accel->z) * 180.0f / M_PI;

    // Yaw rate comes directly from gyro Z-axis
    orient->yaw_rate = gyro->z;
}
void SetMPUOrigin(MPU6050_Handle *hmpu) {
    if (MPU6050_ReadAllData(hmpu) == MPU6050_OK) {
        hmpu->origin.accel = hmpu->scaledAccel;
        hmpu->origin.gyro = hmpu->scaledGyro;
        CalculateOrientation(&hmpu->scaledAccel, &hmpu->scaledGyro, &hmpu->origin.orient);
        hmpu->origin_set = true;
        UART_Print("MPU6050 origin set");
    }
}
bool HasMovedFromOrigin(MPU6050_Handle *hmpu) {
    if (!hmpu->origin_set) return false;

    // Thresholds - adjust these as needed
    const float ACCEL_THRESHOLD = 0.2f;   // 0.2g deviation
    const float GYRO_THRESHOLD = 2.0f;    // 2°/s deviation
    const float ORIENT_THRESHOLD = 5.0f;  // 5° deviation
    const float RETURN_THRESHOLD = 0.5f;  // 0.5g deviation for return (hysteresis)

    // Calculate differences from origin
    float accel_diff = sqrtf(
        powf(hmpu->scaledAccel.x - hmpu->origin.accel.x, 2) +
        powf(hmpu->scaledAccel.y - hmpu->origin.accel.y, 2) +
        powf(hmpu->scaledAccel.z - hmpu->origin.accel.z, 2));

    MPU6050_Orientation current_orient;
    CalculateOrientation(&hmpu->scaledAccel, &hmpu->scaledGyro, &current_orient);

    float orient_diff = sqrtf(
        powf(current_orient.pitch - hmpu->origin.orient.pitch, 2) +
        powf(current_orient.roll - hmpu->origin.orient.roll, 2));

    float gyro_diff = fabs(hmpu->scaledGyro.z - hmpu->origin.gyro.z);

    // Hysteresis logic
    if (!hmpu->currently_away) {
        // Currently at origin - check if we've moved away
        if ((accel_diff > ACCEL_THRESHOLD) ||
            (orient_diff > ORIENT_THRESHOLD) ||
            (gyro_diff > GYRO_THRESHOLD)) {
            hmpu->currently_away = true;
            return true;
        }
    } else {
        // Currently away - check if we've returned
        if ((accel_diff < RETURN_THRESHOLD) &&
            (orient_diff < RETURN_THRESHOLD) &&
            (gyro_diff < RETURN_THRESHOLD)) {
            hmpu->currently_away = false;
            return false;
        }
        return true; // Still away from origin
    }

    return false; // Default case - at origin
}
// Modify UART_PrintMPUData for degree symbols
void UART_PrintMPUData(MPU6050_ScaledData *accel, MPU6050_ScaledData *gyro, float temp) {
    char buffer[128];
    MPU6050_Orientation orient;

    CalculateOrientation(accel, gyro, &orient);

    // Print accelerometer data
    snprintf(buffer, sizeof(buffer), "Accel: X=%.3fg, Y=%.3fg, Z=%.3fg",
            accel->x, accel->y, accel->z);
    UART_Print(buffer);

    // Print gyroscope data
    snprintf(buffer, sizeof(buffer), "Gyro: X=%.3fdeg/s, Y=%.3fdeg/s, Z=%.3fdeg/s",
            gyro->x, gyro->y, gyro->z);
    UART_Print(buffer);

    // Print orientation with degree symbol (ASCII 176)
    snprintf(buffer, sizeof(buffer), "Orientation: Pitch=%.1f%c, Roll=%.1f%c, YawRate=%.1fdeg/s",
            orient.pitch, 176, orient.roll, 176, orient.yaw_rate);
    UART_Print(buffer);

    // Print temperature
    UART_PrintFloat("MPU6050 Temp", temp);
}
MovementDirection DetectMovementDirection(MPU6050_Handle *hmpu) {
    // Calculate differences from origin
    float dx = hmpu->scaledAccel.x - hmpu->origin.accel.x;
    float dy = hmpu->scaledAccel.y - hmpu->origin.accel.y;
    float dz = hmpu->scaledAccel.z - hmpu->origin.accel.z;

    // Get the dominant axis of movement
    float adx = fabsf(dx);
    float ady = fabsf(dy);
    float adz = fabsf(dz);

    // Threshold to consider movement significant (adjust as needed)
    const float MOVEMENT_THRESHOLD = 0.3f;

    if (adx > ady && adx > adz && adx > MOVEMENT_THRESHOLD) {
        return (dx > 0) ? DIRECTION_POSITIVE_X : DIRECTION_NEGATIVE_X;
    } else if (ady > adx && ady > adz && ady > MOVEMENT_THRESHOLD) {
        return (dy > 0) ? DIRECTION_POSITIVE_Y : DIRECTION_NEGATIVE_Y;
    } else if (adz > adx && adz > ady && adz > MOVEMENT_THRESHOLD) {
        return (dz > 0) ? DIRECTION_POSITIVE_Z : DIRECTION_NEGATIVE_Z;
    }

    return DIRECTION_NONE;
}

void ActivateMotorReaction(L298N_HandleTypeDef *hmotor, MovementDirection dir) {
    // Set motor speed (adjust as needed)
    const uint8_t REACTION_SPEED = 70; // 70% speed

    // Stop motors first
    L298N_StopMotor(hmotor, MOTOR_A);
    if (hmotor->mode == L298N_DUAL_CHANNEL) {
        L298N_StopMotor(hmotor, MOTOR_B);
    }

    // Determine which motor(s) to activate based on movement direction
    switch (dir) {
        case DIRECTION_POSITIVE_X:
            // Motor A forward to counteract +X movement
            L298N_SetSpeed(hmotor, MOTOR_A, REACTION_SPEED);
            L298N_SetDirection(hmotor, MOTOR_A, MOTOR_FORWARD);
            break;

        case DIRECTION_NEGATIVE_X:
            // Motor A backward to counteract -X movement
            L298N_SetSpeed(hmotor, MOTOR_A, REACTION_SPEED);
            L298N_SetDirection(hmotor, MOTOR_A, MOTOR_BACKWARD);
            break;

        case DIRECTION_POSITIVE_Y:
            // Motor B forward to counteract +Y movement (if dual channel)
            if (hmotor->mode == L298N_DUAL_CHANNEL) {
                L298N_SetSpeed(hmotor, MOTOR_B, REACTION_SPEED);
                L298N_SetDirection(hmotor, MOTOR_B, MOTOR_FORWARD);
            }
            break;

        case DIRECTION_NEGATIVE_Y:
            // Motor B backward to counteract -Y movement (if dual channel)
            if (hmotor->mode == L298N_DUAL_CHANNEL) {
                L298N_SetSpeed(hmotor, MOTOR_B, REACTION_SPEED);
                L298N_SetDirection(hmotor, MOTOR_B, MOTOR_BACKWARD);
            }
            break;

        case DIRECTION_POSITIVE_Z:
            // Both motors forward to counteract +Z rotation
            L298N_SetSpeed(hmotor, MOTOR_A, REACTION_SPEED);
            L298N_SetDirection(hmotor, MOTOR_A, MOTOR_FORWARD);
            if (hmotor->mode == L298N_DUAL_CHANNEL) {
                L298N_SetSpeed(hmotor, MOTOR_B, REACTION_SPEED);
                L298N_SetDirection(hmotor, MOTOR_B, MOTOR_FORWARD);
            }
            break;

        case DIRECTION_NEGATIVE_Z:
            // Both motors backward to counteract -Z rotation
            L298N_SetSpeed(hmotor, MOTOR_A, REACTION_SPEED);
            L298N_SetDirection(hmotor, MOTOR_A, MOTOR_BACKWARD);
            if (hmotor->mode == L298N_DUAL_CHANNEL) {
                L298N_SetSpeed(hmotor, MOTOR_B, REACTION_SPEED);
                L298N_SetDirection(hmotor, MOTOR_B, MOTOR_BACKWARD);
            }
            break;

        case DIRECTION_NONE:
        default:
            // Shouldn't happen, but stop motors just in case
            L298N_StopMotor(hmotor, MOTOR_A);
            if (hmotor->mode == L298N_DUAL_CHANNEL) {
                L298N_StopMotor(hmotor, MOTOR_B);
            }
            break;
    }
}
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
