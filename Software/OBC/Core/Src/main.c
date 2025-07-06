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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "logging.h"
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
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
char uart_buffer[100];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void process_SD_card(void);
void test_SD_card(void);
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
  MX_USART1_UART_Init();
  MX_SPI1_Init();
  MX_FATFS_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  Log_Init(&huart1);

  // Test logging
  Log_Write(LOG_INFO, "System initialized (Proteus test mode)");
  Log_Write(LOG_INFO, "Starting SD card operations");

  process_SD_card();

  Log_Write(LOG_INFO, "SD card operations completed");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
        // Example of periodic logging
        Log_Write(LOG_DEBUG, "System running in Proteus");
        HAL_Delay(5000);
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
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

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
  HAL_GPIO_WritePin(SD_CS_GPIO_Port, SD_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SD_CS_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SD_CS_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
// In your main.c or a separate timer file
/*extern uint16_t Timer1, Timer2;

 void HAL_SYSTICK_Callback(void)
 {
 if(Timer1 > 0)
 Timer1--;
 if(Timer2 > 0)
 Timer2--;
 }*/

void process_SD_card(void) {
    FATFS FatFs;
    FRESULT fres;
    uint8_t retry_count = 0;
    const uint8_t max_retries = 3;
    char status_msg[60];

    while(retry_count < max_retries) {
        fres = f_mount(&FatFs, "", 1); // 1=Mount now

        if(fres == FR_OK) {
            uint32_t total, free;
            FATFS* fs;
            if(f_getfree("", &free, &fs) == FR_OK) {
                total = (fs->n_fatent - 2) * fs->csize / 2; // Total KB
                free = free * fs->csize / 2; // Free KB
                snprintf(status_msg, sizeof(status_msg),
                    "SD mounted (Free: %lukB/%lukB)", free, total);
                Log_Write(LOG_INFO, status_msg);
                break;
            }
        }

        snprintf(status_msg, sizeof(status_msg),
            "Mount failed (Attempt %d/%d)", retry_count+1, max_retries);
        Log_Write(LOG_WARNING, status_msg);
        HAL_Delay(500);
        retry_count++;
    }

    if(retry_count >= max_retries) {
        Log_Write(LOG_CRITICAL, "SD mount failed permanently");
        return;
    }

    /* ... rest of SD operations ... */
}

void test_SD_card(void) {
	FATFS FatFs;                //Fatfs handle
	FIL fil;                  //File handle
	FRESULT fres;                 //Result after operations
	char buf[100];
	do {
		//Mount the SD Card
		fres = f_mount(&FatFs, "", 1);    //1=mount now
		if (fres != FR_OK) {
			char msg[] = "No SD Card found\r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*) msg, strlen(msg),
					HAL_MAX_DELAY);
			HAL_Delay(1000);
			do {
				fres = f_mount(&FatFs, "", 1);    //1=mount
			} while (fres != FR_OK);
		}

		char msg1[] = "SD Card Mounted Successfully!!!\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) msg1, strlen(msg1),
				HAL_MAX_DELAY);
		HAL_Delay(1000);

		//Read the SD Card Total size and Free Size
		FATFS *pfs;
		DWORD fre_clust;
		uint32_t totalSpace, freeSpace;

		f_getfree("", &fre_clust, &pfs);
		totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
		freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);

		char spaceMsg[100];
		sprintf(spaceMsg, "TotalSpace : %lu bytes, FreeSpace = %lu bytes\r\n",
				totalSpace, freeSpace);
		HAL_UART_Transmit(&huart1, (uint8_t*) spaceMsg, strlen(spaceMsg),
				HAL_MAX_DELAY);
		HAL_Delay(1000);

		//Open the file
		fres = f_open(&fil, "mo3ti.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
		if (fres != FR_OK) {
			char errorMsg[] = "File creation/open Error\r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*) errorMsg, strlen(errorMsg),
					HAL_MAX_DELAY);
			break;
		}

		char writeMsg[] = "Writing data!!!\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) writeMsg, strlen(writeMsg),
				HAL_MAX_DELAY);
		HAL_Delay(1000);

		//write the data
		char uart_buffer[100];
		uint8_t rx_data;
		uint8_t index = 0;

		while (1) {
			if (HAL_UART_Receive(&huart1, &rx_data, 1, 100) == HAL_OK) {
				if (rx_data == '#') {
					uart_buffer[index] = '\0';
					f_close(&fil);
					// Open for reading
					fres = f_open(&fil, "mo3ti.txt", FA_READ);
					f_gets(uart_buffer, sizeof(uart_buffer), &fil);
					char readMsg[] = "Data Read\r\n";
					HAL_UART_Transmit(&huart1, (uint8_t*) readMsg,
							strlen(readMsg), HAL_MAX_DELAY);
					f_close(&fil);
					break;
				} else if (rx_data == '\r') {
					uart_buffer[index] = '\0';
					f_puts(uart_buffer, &fil);
					f_puts("\n", &fil);
					char savedMsg[] = "Saved\r\n";
					HAL_UART_Transmit(&huart1, (uint8_t*) savedMsg,
							strlen(savedMsg), HAL_MAX_DELAY);
					index = 0;
				} else {
					uart_buffer[index++] = rx_data;
					HAL_UART_Transmit(&huart1, &rx_data, 1, 100);
				}
			}
		}

		//close your file
		f_close(&fil);
		//Open the file
		fres = f_open(&fil, "mo3ti.txt", FA_READ);
		if (fres != FR_OK) {
			char errorMsg[] = "File opening Error\r\n";
			HAL_UART_Transmit(&huart1, (uint8_t*) errorMsg, strlen(errorMsg),
					HAL_MAX_DELAY);
			HAL_Delay(1000);
			break;
		}

		//read the data
		f_gets(buf, sizeof(buf), &fil);

		char readDataMsg[] = "Read Data\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) readDataMsg, strlen(readDataMsg),
				HAL_MAX_DELAY);
		HAL_Delay(1000);

		//close your file
		f_close(&fil);
		char closeMsg[] = "Closing File!\r\n";
		HAL_UART_Transmit(&huart1, (uint8_t*) closeMsg, strlen(closeMsg),
				HAL_MAX_DELAY);
		HAL_Delay(1000);

	} while (false);

	//We're done, so de-mount the drive
	f_mount(NULL, "", 0);
	char endMsg[] = "SD Card session ended\r\n";
	HAL_UART_Transmit(&huart1, (uint8_t*) endMsg, strlen(endMsg),
			HAL_MAX_DELAY);
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
