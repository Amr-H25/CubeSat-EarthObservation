/*
 * logging.h
 *
 *  Created on: Apr 21, 2025
 *      Author: Amr_H
 */

#ifndef LOGGING_H
#define LOGGING_H

#include "fatfs.h"

// Log levels
typedef enum {
    LOG_DEBUG,
    LOG_INFO,
    LOG_WARNING,
    LOG_ERROR,
    LOG_CRITICAL
} LogLevel;

// Initialize logging system (simplified version without RTC)
void Log_Init(UART_HandleTypeDef *huart);

// Log a message with timestamp
void Log_Write(LogLevel level, const char *message);

// Set log file name (default is "log.txt")
void Log_SetFilename(const char *filename);

#endif // LOGGING_H
