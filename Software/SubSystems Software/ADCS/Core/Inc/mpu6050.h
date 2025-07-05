/*
 * mpu6050.h
 *
 *  Created on: Apr 24, 2025
 *      Author: Amr_H
 */

/**
  ******************************************************************************
  * @file           : mpu6050.h
  * @brief          : Header for MPU6050 driver
  ******************************************************************************
  * @attention
  *
  * MPU6050 3-Axis Accelerometer and Gyroscope driver for STM32F103
  * Intended for CubeSat proof of concept
  *
  ******************************************************************************
  */

#ifndef MPU6050_H
#define MPU6050_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include <stdbool.h>

/* Defines -------------------------------------------------------------------*/
#define MPU6050_ADDR              0xD0    // MPU6050 Device Address (0x68 << 1)

/* MPU6050 Registers */
#define MPU6050_REG_SMPLRT_DIV    0x19    // Sample Rate Divider
#define MPU6050_REG_CONFIG        0x1A    // Configuration
#define MPU6050_REG_GYRO_CONFIG   0x1B    // Gyroscope Configuration
#define MPU6050_REG_ACCEL_CONFIG  0x1C    // Accelerometer Configuration
#define MPU6050_REG_FIFO_EN       0x23    // FIFO Enable
#define MPU6050_REG_INT_ENABLE    0x38    // Interrupt Enable
#define MPU6050_REG_INT_STATUS    0x3A    // Interrupt Status
#define MPU6050_REG_ACCEL_XOUT_H  0x3B    // X-axis accelerometer high byte
#define MPU6050_REG_ACCEL_XOUT_L  0x3C    // X-axis accelerometer low byte
#define MPU6050_REG_ACCEL_YOUT_H  0x3D    // Y-axis accelerometer high byte
#define MPU6050_REG_ACCEL_YOUT_L  0x3E    // Y-axis accelerometer low byte
#define MPU6050_REG_ACCEL_ZOUT_H  0x3F    // Z-axis accelerometer high byte
#define MPU6050_REG_ACCEL_ZOUT_L  0x40    // Z-axis accelerometer low byte
#define MPU6050_REG_TEMP_OUT_H    0x41    // Temperature high byte
#define MPU6050_REG_TEMP_OUT_L    0x42    // Temperature low byte
#define MPU6050_REG_GYRO_XOUT_H   0x43    // X-axis gyroscope high byte
#define MPU6050_REG_GYRO_XOUT_L   0x44    // X-axis gyroscope low byte
#define MPU6050_REG_GYRO_YOUT_H   0x45    // Y-axis gyroscope high byte
#define MPU6050_REG_GYRO_YOUT_L   0x46    // Y-axis gyroscope low byte
#define MPU6050_REG_GYRO_ZOUT_H   0x47    // Z-axis gyroscope high byte
#define MPU6050_REG_GYRO_ZOUT_L   0x48    // Z-axis gyroscope low byte
#define MPU6050_REG_PWR_MGMT_1    0x6B    // Power Management 1
#define MPU6050_REG_PWR_MGMT_2    0x6C    // Power Management 2
#define MPU6050_REG_WHO_AM_I      0x75    // Who Am I (Device ID)

/* Configuration Values */
#define MPU6050_DEVICE_ID         0x68    // Device ID returned by WHO_AM_I register

/* Gyroscope Scale Selection */
#define MPU6050_GYRO_FS_250       0x00    // ±250 °/s
#define MPU6050_GYRO_FS_500       0x08    // ±500 °/s
#define MPU6050_GYRO_FS_1000      0x10    // ±1000 °/s
#define MPU6050_GYRO_FS_2000      0x18    // ±2000 °/s

/* Accelerometer Scale Selection */
#define MPU6050_ACCEL_FS_2G       0x00    // ±2g
#define MPU6050_ACCEL_FS_4G       0x08    // ±4g
#define MPU6050_ACCEL_FS_8G       0x10    // ±8g
#define MPU6050_ACCEL_FS_16G      0x18    // ±16g

/* Digital Low Pass Filter (DLPF) Configuration */
#define MPU6050_DLPF_BW_256       0x00    // Bandwidth 256Hz
#define MPU6050_DLPF_BW_188       0x01    // Bandwidth 188Hz
#define MPU6050_DLPF_BW_98        0x02    // Bandwidth 98Hz
#define MPU6050_DLPF_BW_42        0x03    // Bandwidth 42Hz
#define MPU6050_DLPF_BW_20        0x04    // Bandwidth 20Hz
#define MPU6050_DLPF_BW_10        0x05    // Bandwidth 10Hz
#define MPU6050_DLPF_BW_5         0x06    // Bandwidth 5Hz

/* Status Codes */
typedef enum {
    MPU6050_OK          = 0x00,
    MPU6050_ERROR       = 0x01,
    MPU6050_BUSY        = 0x02,
    MPU6050_TIMEOUT     = 0x03,
    MPU6050_NO_DEVICE   = 0x04,
    MPU6050_NOT_READY   = 0x05
} MPU6050_Status;

/* Data Structures */
typedef struct {
    int16_t x;
    int16_t y;
    int16_t z;
} MPU6050_RawData;

typedef struct {
    float x;
    float y;
    float z;
} MPU6050_ScaledData;

typedef struct {
    float pitch;    // Rotation around Y-axis (degrees)
    float roll;     // Rotation around X-axis (degrees)
    float yaw_rate; // Rotation rate around Z-axis (degrees/second)
} MPU6050_Orientation;

typedef struct {
    MPU6050_ScaledData accel;
    MPU6050_ScaledData gyro;
    MPU6050_Orientation orient;
} MPU6050_Origin;

typedef struct {
    I2C_HandleTypeDef *hi2c;          // I2C handle pointer
    uint8_t address;                   // Device address
    uint8_t gyroScale;                 // Current gyroscope scale
    uint8_t accelScale;                // Current accelerometer scale
    MPU6050_RawData rawAccel;          // Raw accelerometer data
    MPU6050_RawData rawGyro;           // Raw gyroscope data
    MPU6050_ScaledData scaledAccel;    // Scaled accelerometer data (g)
    MPU6050_ScaledData scaledGyro;     // Scaled gyroscope data (deg/s)
    float temperature;                 // Temperature (°C)
    float gyroScaleFactor;             // Scale factor for gyroscope
    float accelScaleFactor;            // Scale factor for accelerometer
    uint16_t calibDelay;               // Calibration delay in milliseconds
    MPU6050_Origin origin;
    bool origin_set;
    bool currently_away;
} MPU6050_Handle;

/* Function Prototypes */
MPU6050_Status MPU6050_Init(MPU6050_Handle *hdev, I2C_HandleTypeDef *hi2c);
MPU6050_Status MPU6050_SetGyroScale(MPU6050_Handle *hdev, uint8_t scale);
MPU6050_Status MPU6050_SetAccelScale(MPU6050_Handle *hdev, uint8_t scale);
MPU6050_Status MPU6050_SetDLPF(MPU6050_Handle *hdev, uint8_t bandwidth);
MPU6050_Status MPU6050_ReadAllData(MPU6050_Handle *hdev);
MPU6050_Status MPU6050_ReadAccelerometerData(MPU6050_Handle *hdev);
MPU6050_Status MPU6050_ReadGyroscopeData(MPU6050_Handle *hdev);
MPU6050_Status MPU6050_ReadTemperature(MPU6050_Handle *hdev);
MPU6050_Status MPU6050_Calibrate(MPU6050_Handle *hdev, uint16_t numSamples);
bool MPU6050_IsDataReady(MPU6050_Handle *hdev);
void MPU6050_Print(MPU6050_Handle *hdev, UART_HandleTypeDef *huart);

#ifdef __cplusplus
}
#endif

#endif /* MPU6050_H */
