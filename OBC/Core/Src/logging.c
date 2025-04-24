/*
 * logging.c
 *
 *  Created on: Apr 21, 2025
 *      Author: Amr_H
 */

#include "logging.h"
#include "ds1307.h"
#include <string.h>
#include <stdio.h>

static char log_filename[32] = "log.txt";
static UART_HandleTypeDef *_huart;
static uint32_t dummy_timestamp = 0;

// RTC status flag
static bool rtc_available = false;

void Log_Init(UART_HandleTypeDef *huart) {
    _huart = huart;
    dummy_timestamp = 0;

    // Try to initialize RTC (but don't fail if it's not available)
    #ifdef USE_REAL_RTC
    if (_hi2c != NULL) {
        DS1307_Init(&hi2c1, huart);
        rtc_available = DS1307_IsRunning();
        if (!rtc_available) {
            Log_Write(LOG_WARNING, "RTC not running, using dummy timestamps");
        } else {
            Log_Write(LOG_INFO, "RTC initialized successfully");
        }
    }
    #endif
}

void Log_SetFilename(const char *filename) {
    strncpy(log_filename, filename, sizeof(log_filename)-1);
    log_filename[sizeof(log_filename)-1] = '\0';
}

// Convert log level to string
static const char* _log_level_to_string(LogLevel level) {
    switch (level) {
        case LOG_DEBUG:    return "DEBUG";
        case LOG_INFO:     return "INFO";
        case LOG_WARNING:  return "WARN";
        case LOG_ERROR:    return "ERROR";
        case LOG_CRITICAL: return "CRITICAL";
        default:           return "UNKNOWN";
    }
}

// Get current timestamp (either from RTC or dummy)
static void _get_timestamp(char *timestamp_buf, size_t buf_size) {
    #ifdef USE_REAL_RTC
    if (rtc_available) {
        DS1307_Time time;
        DS1307_Date date;
        if (DS1307_GetTime(&time) {
            DS1307_GetDate(&date);
            snprintf(timestamp_buf, buf_size, "%02d/%02d/20%02d %02d:%02d:%02d",
                    date.date, date.month, date.year,
                    time.hours, time.minutes, time.seconds);
            return;
        }
    }
    #endif

    // Fallback to dummy timestamp if RTC not available
    uint32_t hours = (dummy_timestamp / 3600) % 24;
    uint32_t minutes = (dummy_timestamp / 60) % 60;
    uint32_t seconds = dummy_timestamp % 60;
    dummy_timestamp++;  // Increment for next log

    snprintf(timestamp_buf, buf_size, "DUMMY %02lu:%02lu:%02lu",
            hours, minutes, seconds);
}

void Log_Write(LogLevel level, const char *message) {
    FATFS fs;
    FIL fil;
    FRESULT fres;
    char buffer[256];
    char timestamp[32];

    // Get timestamp
    _get_timestamp(timestamp, sizeof(timestamp));

    // Format log message
    snprintf(buffer, sizeof(buffer), "[%s][%s] %s\r\n",
            timestamp,
            _log_level_to_string(level),
            message);

    // Mount the filesystem
    fres = f_mount(&fs, "", 1);
    if (fres != FR_OK) {
        if (_huart) {
            HAL_UART_Transmit(_huart, (uint8_t*)"Failed to mount filesystem\r\n",
                             strlen("Failed to mount filesystem\r\n"), HAL_MAX_DELAY);
        }
        return;
    }

    // Open or create the log file (append mode)
    fres = f_open(&fil, log_filename, FA_WRITE | FA_OPEN_ALWAYS);
    if (fres == FR_OK) {
        // Seek to end of file for appending
        f_lseek(&fil, f_size(&fil));

        // Write the log message
        UINT bytes_written;
        fres = f_write(&fil, buffer, strlen(buffer), &bytes_written);
        if (fres != FR_OK || bytes_written != strlen(buffer)) {
            if (_huart) {
                HAL_UART_Transmit(_huart, (uint8_t*)"Failed to write to log file\r\n",
                                 strlen("Failed to write to log file\r\n"), HAL_MAX_DELAY);
            }
        }

        // Close the file
        f_close(&fil);
    } else {
        if (_huart) {
            HAL_UART_Transmit(_huart, (uint8_t*)"Failed to open log file\r\n",
                             strlen("Failed to open log file\r\n"), HAL_MAX_DELAY);
        }
    }

    // Unmount
    f_mount(NULL, "", 0);

    // Also output to UART for debugging
    if (_huart) {
        HAL_UART_Transmit(_huart, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);
    }
}
