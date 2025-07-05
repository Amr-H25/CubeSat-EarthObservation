/*
 * ds1307.c
 *
 *  Created on: Apr 20, 2025
 *      Author: Amr_H
 */


#include "DS1307.h"
#include <stdio.h>
#include <string.h>

// Private variables
static I2C_HandleTypeDef *_hi2c;
static UART_HandleTypeDef *_huart;
static char _uart_buf[100];

// Private function prototypes
static uint8_t _dec2bcd(uint8_t dec);
static uint8_t _bcd2dec(uint8_t bcd);
static bool _validate_time(DS1307_Time *time);
static bool _validate_date(DS1307_Date *date);
static void _debug_print(const char *message);

// Initialization function
void DS1307_Init(I2C_HandleTypeDef *hi2c, UART_HandleTypeDef *huart) {
    _hi2c = hi2c;
    _huart = huart;
    _debug_print("DS1307 Driver Initialized\r\n");
}

// Check if clock is running
bool DS1307_IsRunning(void) {
    uint8_t seconds;
    if(HAL_I2C_Mem_Read(_hi2c, DS1307_I2C_ADDR, DS1307_REG_SECONDS, 1, &seconds, 1, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error reading clock status\r\n");
        return false;
    }
    return !(seconds & DS1307_CH_BIT);
}

// Start the clock
void DS1307_StartClock(void) {
    uint8_t seconds;
    if(HAL_I2C_Mem_Read(_hi2c, DS1307_I2C_ADDR, DS1307_REG_SECONDS, 1, &seconds, 1, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error reading seconds register\r\n");
        return;
    }

    seconds &= ~DS1307_CH_BIT;  // Clear CH bit

    if(HAL_I2C_Mem_Write(_hi2c, DS1307_I2C_ADDR, DS1307_REG_SECONDS, 1, &seconds, 1, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error starting clock\r\n");
    } else {
        _debug_print("Clock started\r\n");
    }
}

// Stop the clock
void DS1307_StopClock(void) {
    uint8_t seconds;
    if(HAL_I2C_Mem_Read(_hi2c, DS1307_I2C_ADDR, DS1307_REG_SECONDS, 1, &seconds, 1, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error reading seconds register\r\n");
        return;
    }

    seconds |= DS1307_CH_BIT;  // Set CH bit

    if(HAL_I2C_Mem_Write(_hi2c, DS1307_I2C_ADDR, DS1307_REG_SECONDS, 1, &seconds, 1, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error stopping clock\r\n");
    } else {
        _debug_print("Clock stopped\r\n");
    }
}

// Get current time
bool DS1307_GetTime(DS1307_Time *time) {
    uint8_t data[3];

    if(HAL_I2C_Mem_Read(_hi2c, DS1307_I2C_ADDR, DS1307_REG_SECONDS, 1, data, 3, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error reading time registers\r\n");
        return false;
    }

    time->seconds = _bcd2dec(data[0] & 0x7F);
    time->minutes = _bcd2dec(data[1] & 0x7F);

    // Handle hours format
    if(data[2] & DS1307_12HOUR_BIT) {
        time->format = DS1307_FORMAT_12H;
        time->is_pm = (data[2] & DS1307_PM_BIT) ? true : false;
        time->hours = _bcd2dec(data[2] & 0x1F);  // Mask out 12-hour and PM bits
    } else {
        time->format = DS1307_FORMAT_24H;
        time->is_pm = false;
        time->hours = _bcd2dec(data[2] & 0x3F);  // Mask out 24-hour bits
    }

    return true;
}

// Set current time
bool DS1307_SetTime(DS1307_Time *time) {
    if(!_validate_time(time)) {
        _debug_print("Invalid time format\r\n");
        return false;
    }

    uint8_t data[3];
    uint8_t seconds_reg;

    // First read the seconds register to preserve the CH bit
    if(HAL_I2C_Mem_Read(_hi2c, DS1307_I2C_ADDR, DS1307_REG_SECONDS, 1, &seconds_reg, 1, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error reading seconds register\r\n");
        return false;
    }

    data[0] = _dec2bcd(time->seconds) | (seconds_reg & DS1307_CH_BIT);  // Preserve CH bit
    data[1] = _dec2bcd(time->minutes);

    if(time->format == DS1307_FORMAT_12H) {
        data[2] = _dec2bcd(time->hours) | DS1307_12HOUR_BIT;
        if(time->is_pm) {
            data[2] |= DS1307_PM_BIT;
        }
    } else {
        data[2] = _dec2bcd(time->hours);
    }

    if(HAL_I2C_Mem_Write(_hi2c, DS1307_I2C_ADDR, DS1307_REG_SECONDS, 1, data, 3, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error setting time\r\n");
        return false;
    }

    return true;
}

// Get current date
bool DS1307_GetDate(DS1307_Date *date) {
    uint8_t data[4];

    if(HAL_I2C_Mem_Read(_hi2c, DS1307_I2C_ADDR, DS1307_REG_DAY, 1, data, 4, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error reading date registers\r\n");
        return false;
    }

    date->day = _bcd2dec(data[0]);
    date->date = _bcd2dec(data[1]);
    date->month = _bcd2dec(data[2]);
    date->year = _bcd2dec(data[3]);

    return true;
}

// Set current date
bool DS1307_SetDate(DS1307_Date *date) {
    if(!_validate_date(date)) {
        _debug_print("Invalid date format\r\n");
        return false;
    }

    uint8_t data[4];

    data[0] = _dec2bcd(date->day);
    data[1] = _dec2bcd(date->date);
    data[2] = _dec2bcd(date->month);
    data[3] = _dec2bcd(date->year);

    if(HAL_I2C_Mem_Write(_hi2c, DS1307_I2C_ADDR, DS1307_REG_DAY, 1, data, 4, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error setting date\r\n");
        return false;
    }

    return true;
}

// Enable square wave output
void DS1307_EnableSquareWave(DS1307_SQW_Rate rate) {
    uint8_t control = DS1307_SQW_ENABLE_BIT | (rate & DS1307_SQW_RATE_MASK);

    if(HAL_I2C_Mem_Write(_hi2c, DS1307_I2C_ADDR, DS1307_REG_CONTROL, 1, &control, 1, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error enabling square wave\r\n");
    } else {
        snprintf(_uart_buf, sizeof(_uart_buf), "Square wave enabled at %dHz\r\n",
                rate == DS1307_SQW_1HZ ? 1 :
                rate == DS1307_SQW_4KHZ ? 4096 :
                rate == DS1307_SQW_8KHZ ? 8192 : 32768);
        _debug_print(_uart_buf);
    }
}

// Disable square wave output
void DS1307_DisableSquareWave(void) {
    uint8_t control = 0x00;

    if(HAL_I2C_Mem_Write(_hi2c, DS1307_I2C_ADDR, DS1307_REG_CONTROL, 1, &control, 1, HAL_MAX_DELAY) != HAL_OK) {
        _debug_print("Error disabling square wave\r\n");
    } else {
        _debug_print("Square wave disabled\r\n");
    }
}

// Print time to UART
void DS1307_PrintTime(DS1307_Time *time) {
    if(time->format == DS1307_FORMAT_12H) {
        snprintf(_uart_buf, sizeof(_uart_buf), "Time: %02d:%02d:%02d %s\r\n",
                time->hours, time->minutes, time->seconds,
                time->is_pm ? "PM" : "AM");
    } else {
        snprintf(_uart_buf, sizeof(_uart_buf), "Time: %02d:%02d:%02d\r\n",
                time->hours, time->minutes, time->seconds);
    }
    _debug_print(_uart_buf);
}

// Print date to UART
void DS1307_PrintDate(DS1307_Date *date) {
    const char *days[] = {"", "Sun", "Mon", "Tue", "Wed", "Thu", "Fri", "Sat"};
    const char *months[] = {"", "Jan", "Feb", "Mar", "Apr", "May", "Jun",
                           "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};

    snprintf(_uart_buf, sizeof(_uart_buf), "Date: %s %02d %s 20%02d\r\n",
            days[date->day], date->date, months[date->month], date->year);
    _debug_print(_uart_buf);
}

// Convert decimal to BCD
static uint8_t _dec2bcd(uint8_t dec) {
    return ((dec / 10) << 4) | (dec % 10);
}

// Convert BCD to decimal
static uint8_t _bcd2dec(uint8_t bcd) {
    return ((bcd >> 4) * 10) + (bcd & 0x0F);
}

// Validate time structure
static bool _validate_time(DS1307_Time *time) {
    if(time->seconds > 59 || time->minutes > 59) {
        return false;
    }

    if(time->format == DS1307_FORMAT_12H) {
        if(time->hours < 1 || time->hours > 12) {
            return false;
        }
    } else {
        if(time->hours > 23) {
            return false;
        }
    }

    return true;
}

// Validate date structure
static bool _validate_date(DS1307_Date *date) {
    if(date->day < 1 || date->day > 7) {
        return false;
    }

    if(date->date < 1 || date->date > 31) {
        return false;
    }

    if(date->month < 1 || date->month > 12) {
        return false;
    }

    if(date->year > 99) {
        return false;
    }

    // Add more specific checks for month lengths if needed

    return true;
}

// Debug print function
static void _debug_print(const char *message) {
    if(_huart != NULL) {
        HAL_UART_Transmit(_huart, (uint8_t*)message, strlen(message), HAL_MAX_DELAY);
    }
}
