/*
 * mpu6050.c
 *
 *  Created on: Apr 24, 2025
 *      Author: Amr_H
 */
/********************************************************************************
  * @file           : mpu6050.c
  * @brief          : MPU6050 driver implementation
  ******************************************************************************
  * @attention
  *
  * MPU6050 3-Axis Accelerometer and Gyroscope driver for STM32F103
  * Intended for CubeSat proof of concept
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "mpu6050.h"
#include <string.h>
#include <stdio.h>
#include <math.h>

/* Private function prototypes -----------------------------------------------*/
static MPU6050_Status MPU6050_ReadRegister(MPU6050_Handle *hdev, uint8_t reg, uint8_t *data);
static MPU6050_Status MPU6050_ReadRegisters(MPU6050_Handle *hdev, uint8_t reg, uint8_t *data, uint8_t length);
static MPU6050_Status MPU6050_WriteRegister(MPU6050_Handle *hdev, uint8_t reg, uint8_t data);

/**
 * @brief Initialize the MPU6050
 * @param hdev MPU6050 handle
 * @param hi2c I2C handle pointer
 * @return Status
 */
MPU6050_Status MPU6050_Init(MPU6050_Handle *hdev, I2C_HandleTypeDef *hi2c) {
    uint8_t check;
    MPU6050_Status status;

    /* Store I2C handle */
    hdev->hi2c = hi2c;
    hdev->address = MPU6050_ADDR;
    hdev->calibDelay = 50; // Default calibration delay 50ms

    /* Initialize raw and scaled data structures */
    memset(&hdev->rawAccel, 0, sizeof(hdev->rawAccel));
    memset(&hdev->rawGyro, 0, sizeof(hdev->rawGyro));
    memset(&hdev->scaledAccel, 0, sizeof(hdev->scaledAccel));
    memset(&hdev->scaledGyro, 0, sizeof(hdev->scaledGyro));
    hdev->temperature = 0.0f;

    /* Check if device is responding */
    status = MPU6050_ReadRegister(hdev, MPU6050_REG_WHO_AM_I, &check);
    if (status != MPU6050_OK || check != MPU6050_DEVICE_ID) {
        return MPU6050_NO_DEVICE;
    }

    /* Wake up the MPU6050 */
    status = MPU6050_WriteRegister(hdev, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (status != MPU6050_OK) {
        return status;
    }

    /* Set sample rate divider */
    status = MPU6050_WriteRegister(hdev, MPU6050_REG_SMPLRT_DIV, 0x07); // 1kHz / (1 + 7) = 125Hz
    if (status != MPU6050_OK) {
        return status;
    }

    /* Set DLPF bandwidth to 42Hz */
    status = MPU6050_SetDLPF(hdev, MPU6050_DLPF_BW_42);
    if (status != MPU6050_OK) {
        return status;
    }

    /* Set gyroscope scale to ±500 °/s */
    status = MPU6050_SetGyroScale(hdev, MPU6050_GYRO_FS_500);
    if (status != MPU6050_OK) {
        return status;
    }

    /* Set accelerometer scale to ±4g */
    status = MPU6050_SetAccelScale(hdev, MPU6050_ACCEL_FS_4G);
    if (status != MPU6050_OK) {
        return status;
    }

    return MPU6050_OK;
}

/**
 * @brief Set gyroscope full scale range
 * @param hdev MPU6050 handle
 * @param scale Scale selection (MPU6050_GYRO_FS_xxx)
 * @return Status
 */
MPU6050_Status MPU6050_SetGyroScale(MPU6050_Handle *hdev, uint8_t scale) {
    MPU6050_Status status;

    /* Set gyroscope scale */
    status = MPU6050_WriteRegister(hdev, MPU6050_REG_GYRO_CONFIG, scale);
    if (status != MPU6050_OK) {
        return status;
    }

    /* Store current scale selection */
    hdev->gyroScale = scale;

    /* Calculate scale factor based on selection */
    switch (scale) {
        case MPU6050_GYRO_FS_250:
            hdev->gyroScaleFactor = 131.0f; // 131 LSB/(°/s)
            break;
        case MPU6050_GYRO_FS_500:
            hdev->gyroScaleFactor = 65.5f;  // 65.5 LSB/(°/s)
            break;
        case MPU6050_GYRO_FS_1000:
            hdev->gyroScaleFactor = 32.8f;  // 32.8 LSB/(°/s)
            break;
        case MPU6050_GYRO_FS_2000:
            hdev->gyroScaleFactor = 16.4f;  // 16.4 LSB/(°/s)
            break;
        default:
            hdev->gyroScaleFactor = 65.5f;  // Default to ±500 °/s
            break;
    }

    return MPU6050_OK;
}

/**
 * @brief Set accelerometer full scale range
 * @param hdev MPU6050 handle
 * @param scale Scale selection (MPU6050_ACCEL_FS_xxx)
 * @return Status
 */
MPU6050_Status MPU6050_SetAccelScale(MPU6050_Handle *hdev, uint8_t scale) {
    MPU6050_Status status;

    /* Set accelerometer scale */
    status = MPU6050_WriteRegister(hdev, MPU6050_REG_ACCEL_CONFIG, scale);
    if (status != MPU6050_OK) {
        return status;
    }

    /* Store current scale selection */
    hdev->accelScale = scale;

    /* Calculate scale factor based on selection */
    switch (scale) {
        case MPU6050_ACCEL_FS_2G:
            hdev->accelScaleFactor = 16384.0f; // 16384 LSB/g
            break;
        case MPU6050_ACCEL_FS_4G:
            hdev->accelScaleFactor = 8192.0f;  // 8192 LSB/g
            break;
        case MPU6050_ACCEL_FS_8G:
            hdev->accelScaleFactor = 4096.0f;  // 4096 LSB/g
            break;
        case MPU6050_ACCEL_FS_16G:
            hdev->accelScaleFactor = 2048.0f;  // 2048 LSB/g
            break;
        default:
            hdev->accelScaleFactor = 8192.0f;  // Default to ±4g
            break;
    }

    return MPU6050_OK;
}

/**
 * @brief Set digital low pass filter bandwidth
 * @param hdev MPU6050 handle
 * @param bandwidth Bandwidth selection (MPU6050_DLPF_BW_xxx)
 * @return Status
 */
MPU6050_Status MPU6050_SetDLPF(MPU6050_Handle *hdev, uint8_t bandwidth) {
    return MPU6050_WriteRegister(hdev, MPU6050_REG_CONFIG, bandwidth);
}

/**
 * @brief Read all data from MPU6050 (accelerometer, gyroscope, temperature)
 * @param hdev MPU6050 handle
 * @return Status
 */
MPU6050_Status MPU6050_ReadAllData(MPU6050_Handle *hdev) {
    uint8_t data[14];
    MPU6050_Status status;

    /* Read all sensor data starting from ACCEL_XOUT_H (14 bytes total) */
    status = MPU6050_ReadRegisters(hdev, MPU6050_REG_ACCEL_XOUT_H, data, 14);
    if (status != MPU6050_OK) {
        return status;
    }

    /* Parse accelerometer data */
    hdev->rawAccel.x = (int16_t)((data[0] << 8) | data[1]);
    hdev->rawAccel.y = (int16_t)((data[2] << 8) | data[3]);
    hdev->rawAccel.z = (int16_t)((data[4] << 8) | data[5]);

    /* Parse temperature data */
    int16_t rawTemp = (int16_t)((data[6] << 8) | data[7]);
    hdev->temperature = (float)rawTemp / 340.0f + 36.53f; // MPU6050 formula from datasheet

    /* Parse gyroscope data */
    hdev->rawGyro.x = (int16_t)((data[8] << 8) | data[9]);
    hdev->rawGyro.y = (int16_t)((data[10] << 8) | data[11]);
    hdev->rawGyro.z = (int16_t)((data[12] << 8) | data[13]);

    /* Convert to scaled values */
    hdev->scaledAccel.x = (float)hdev->rawAccel.x / hdev->accelScaleFactor;
    hdev->scaledAccel.y = (float)hdev->rawAccel.y / hdev->accelScaleFactor;
    hdev->scaledAccel.z = (float)hdev->rawAccel.z / hdev->accelScaleFactor;

    hdev->scaledGyro.x = (float)hdev->rawGyro.x / hdev->gyroScaleFactor;
    hdev->scaledGyro.y = (float)hdev->rawGyro.y / hdev->gyroScaleFactor;
    hdev->scaledGyro.z = (float)hdev->rawGyro.z / hdev->gyroScaleFactor;

    return MPU6050_OK;
}

/**
 * @brief Read accelerometer data only
 * @param hdev MPU6050 handle
 * @return Status
 */
MPU6050_Status MPU6050_ReadAccelerometerData(MPU6050_Handle *hdev) {
    uint8_t data[6];
    MPU6050_Status status;

    /* Read accelerometer data (6 bytes total) */
    status = MPU6050_ReadRegisters(hdev, MPU6050_REG_ACCEL_XOUT_H, data, 6);
    if (status != MPU6050_OK) {
        return status;
    }

    /* Parse data */
    hdev->rawAccel.x = (int16_t)((data[0] << 8) | data[1]);
    hdev->rawAccel.y = (int16_t)((data[2] << 8) | data[3]);
    hdev->rawAccel.z = (int16_t)((data[4] << 8) | data[5]);

    /* Convert to scaled values */
    hdev->scaledAccel.x = (float)hdev->rawAccel.x / hdev->accelScaleFactor;
    hdev->scaledAccel.y = (float)hdev->rawAccel.y / hdev->accelScaleFactor;
    hdev->scaledAccel.z = (float)hdev->rawAccel.z / hdev->accelScaleFactor;

    return MPU6050_OK;
}

/**
 * @brief Read gyroscope data only
 * @param hdev MPU6050 handle
 * @return Status
 */
MPU6050_Status MPU6050_ReadGyroscopeData(MPU6050_Handle *hdev) {
    uint8_t data[6];
    MPU6050_Status status;

    /* Read gyroscope data (6 bytes total) */
    status = MPU6050_ReadRegisters(hdev, MPU6050_REG_GYRO_XOUT_H, data, 6);
    if (status != MPU6050_OK) {
        return status;
    }

    /* Parse data */
    hdev->rawGyro.x = (int16_t)((data[0] << 8) | data[1]);
    hdev->rawGyro.y = (int16_t)((data[2] << 8) | data[3]);
    hdev->rawGyro.z = (int16_t)((data[4] << 8) | data[5]);

    /* Convert to scaled values */
    hdev->scaledGyro.x = (float)hdev->rawGyro.x / hdev->gyroScaleFactor;
    hdev->scaledGyro.y = (float)hdev->rawGyro.y / hdev->gyroScaleFactor;
    hdev->scaledGyro.z = (float)hdev->rawGyro.z / hdev->gyroScaleFactor;

    return MPU6050_OK;
}

/**
 * @brief Read temperature data only
 * @param hdev MPU6050 handle
 * @return Status
 */
MPU6050_Status MPU6050_ReadTemperature(MPU6050_Handle *hdev) {
    uint8_t data[2];
    MPU6050_Status status;

    /* Read temperature data (2 bytes) */
    status = MPU6050_ReadRegisters(hdev, MPU6050_REG_TEMP_OUT_H, data, 2);
    if (status != MPU6050_OK) {
        return status;
    }

    /* Parse data */
    int16_t rawTemp = (int16_t)((data[0] << 8) | data[1]);
    hdev->temperature = (float)rawTemp / 340.0f + 36.53f; // MPU6050 formula from datasheet

    return MPU6050_OK;
}

/**
 * @brief Check if new data is ready from MPU6050
 * @param hdev MPU6050 handle
 * @return true if data is ready, false otherwise
 */
bool MPU6050_IsDataReady(MPU6050_Handle *hdev) {
    uint8_t status;

    if (MPU6050_ReadRegister(hdev, MPU6050_REG_INT_STATUS, &status) != MPU6050_OK) {
        return false;
    }

    return (status & 0x01) ? true : false; // Check data ready bit
}

/**
 * @brief Calibrate the MPU6050 by calculating gyro offset
 * @param hdev MPU6050 handle
 * @param numSamples Number of samples to use for calibration
 * @return Status
 */
MPU6050_Status MPU6050_Calibrate(MPU6050_Handle *hdev, uint16_t numSamples) {
    int32_t gyroXSum = 0, gyroYSum = 0, gyroZSum = 0;
    MPU6050_Status status;

    /* Collect multiple samples for better accuracy */
    for (uint16_t i = 0; i < numSamples; i++) {
        status = MPU6050_ReadGyroscopeData(hdev);
        if (status != MPU6050_OK) {
            return status;
        }

        gyroXSum += hdev->rawGyro.x;
        gyroYSum += hdev->rawGyro.y;
        gyroZSum += hdev->rawGyro.z;

        /* Short delay between measurements */
        HAL_Delay(hdev->calibDelay);
    }

    /* Calculate average offsets */
    int16_t gyroXOffset = gyroXSum / numSamples;
    int16_t gyroYOffset = gyroYSum / numSamples;
    int16_t gyroZOffset = gyroZSum / numSamples;

    /* Apply offsets to subsequent readings */
    hdev->rawGyro.x -= gyroXOffset;
    hdev->rawGyro.y -= gyroYOffset;
    hdev->rawGyro.z -= gyroZOffset;

    return MPU6050_OK;
}

/**
 * @brief Print MPU6050 data to UART
 * @param hdev MPU6050 handle
 * @param huart UART handle pointer
 */
void MPU6050_Print(MPU6050_Handle *hdev, UART_HandleTypeDef *huart) {
    char buffer[256];

    sprintf(buffer, "\r\n--------- MPU6050 DATA ---------\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "Accel (g): X=%.3f, Y=%.3f, Z=%.3f\r\n",
            hdev->scaledAccel.x, hdev->scaledAccel.y, hdev->scaledAccel.z);
    HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "Gyro (deg/s): X=%.3f, Y=%.3f, Z=%.3f\r\n",
            hdev->scaledGyro.x, hdev->scaledGyro.y, hdev->scaledGyro.z);
    HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "Temperature: %.2f C\r\n", hdev->temperature);
    HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), 100);

    sprintf(buffer, "--------------------------------\r\n");
    HAL_UART_Transmit(huart, (uint8_t*)buffer, strlen(buffer), 100);
}

/* Private Functions --------------------------------------------------------*/

/**
 * @brief Read a single register from MPU6050
 * @param hdev MPU6050 handle
 * @param reg Register address
 * @param data Pointer to data buffer
 * @return Status
 */
static MPU6050_Status MPU6050_ReadRegister(MPU6050_Handle *hdev, uint8_t reg, uint8_t *data) {
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(hdev->hi2c, hdev->address, reg, I2C_MEMADD_SIZE_8BIT, data, 1, 100);

    return (status == HAL_OK) ? MPU6050_OK : MPU6050_ERROR;
}

/**
 * @brief Read multiple registers from MPU6050
 * @param hdev MPU6050 handle
 * @param reg Starting register address
 * @param data Pointer to data buffer
 * @param length Number of bytes to read
 * @return Status
 */
static MPU6050_Status MPU6050_ReadRegisters(MPU6050_Handle *hdev, uint8_t reg, uint8_t *data, uint8_t length) {
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Read(hdev->hi2c, hdev->address, reg, I2C_MEMADD_SIZE_8BIT, data, length, 100);

    return (status == HAL_OK) ? MPU6050_OK : MPU6050_ERROR;
}

/**
 * @brief Write a single register to MPU6050
 * @param hdev MPU6050 handle
 * @param reg Register address
 * @param data Data to write
 * @return Status
 */
static MPU6050_Status MPU6050_WriteRegister(MPU6050_Handle *hdev, uint8_t reg, uint8_t data) {
    HAL_StatusTypeDef status;

    status = HAL_I2C_Mem_Write(hdev->hi2c, hdev->address, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 100);

    return (status == HAL_OK) ? MPU6050_OK : MPU6050_ERROR;
}


