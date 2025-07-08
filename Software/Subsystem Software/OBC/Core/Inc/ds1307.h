/*
 * ds1307.h
 *
 *  Created on: Apr 20, 2025
 *      Author: Amr_H
 */

#ifndef DS1307_H
#define DS1307_H

#include "stm32f1xx_hal.h"  // Adjust based on your STM32 series
#include <stdint.h>
#include <stdbool.h>

// DS1307 I2C Address
#define DS1307_I2C_ADDR         0x68 << 1  // 7-bit address shifted left by 1

// Register addresses
#define DS1307_REG_SECONDS      0x00
#define DS1307_REG_MINUTES      0x01
#define DS1307_REG_HOURS        0x02
#define DS1307_REG_DAY          0x03
#define DS1307_REG_DATE         0x04
#define DS1307_REG_MONTH        0x05
#define DS1307_REG_YEAR         0x06
#define DS1307_REG_CONTROL      0x07

// Bit masks
#define DS1307_CH_BIT           0x80  // Clock Halt bit in seconds register
#define DS1307_12HOUR_BIT       0x40  // 12-hour format bit in hours register
#define DS1307_PM_BIT           0x20  // PM bit in 12-hour format
#define DS1307_SQW_ENABLE_BIT   0x10  // Square wave output enable bit
#define DS1307_SQW_RATE_MASK    0x03  // Square wave rate selection mask

// Square wave output frequencies
typedef enum {
    DS1307_SQW_1HZ = 0x00,
    DS1307_SQW_4KHZ = 0x01,
    DS1307_SQW_8KHZ = 0x02,
    DS1307_SQW_32KHZ = 0x03
} DS1307_SQW_Rate;

// Time format
typedef enum {
    DS1307_FORMAT_24H = 0,
    DS1307_FORMAT_12H
} DS1307_TimeFormat;

// Time structure
typedef struct {
    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    DS1307_TimeFormat format;
    bool is_pm;
} DS1307_Time;

// Date structure
typedef struct {
    uint8_t day;    // Day of week (1-7)
    uint8_t date;   // Day of month (1-31)
    uint8_t month;  // Month (1-12)
    uint8_t year;   // Year (00-99)
} DS1307_Date;

// Initialization function
void DS1307_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart);

// Basic RTC functions
bool DS1307_IsRunning(void);
void DS1307_StartClock(void);
void DS1307_StopClock(void);

// Time functions
bool DS1307_GetTime(DS1307_Time *time);
bool DS1307_SetTime(DS1307_Time *time);

// Date functions
bool DS1307_GetDate(DS1307_Date *date);
bool DS1307_SetDate(DS1307_Date *date);

// Control functions
void DS1307_EnableSquareWave(DS1307_SQW_Rate rate);
void DS1307_DisableSquareWave(void);

// Utility functions
void DS1307_PrintTime(DS1307_Time *time);
void DS1307_PrintDate(DS1307_Date *date);

#endif // DS1307_H
