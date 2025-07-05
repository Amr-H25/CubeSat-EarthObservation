/*
 * hc05-driver.c - Fixed Implementation
 *
 *  Created on: Jul 3, 2025
 *      Author: Amr_H
 */

#include "hc05_driver.h"
#include <stdio.h>

extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
extern DMA_HandleTypeDef hdma_usart3_tx;

static volatile bool command_received = false;
static char received_command[COMMAND_BUFFER_SIZE] = {0};
static uint8_t rx_buffer[COMMAND_BUFFER_SIZE] = {0};
static uint8_t rx_single_byte = 0;
static uint16_t rx_index = 0;
static volatile bool led_blink_flag = false;

extern UART_HandleTypeDef huart2;
#define DEBUG_UART &huart2

// Initialize Bluetooth module
BT_StatusTypeDef BT_Init(UART_HandleTypeDef *huart) {
    // Clear buffers
    memset(rx_buffer, 0, COMMAND_BUFFER_SIZE);
    memset(received_command, 0, COMMAND_BUFFER_SIZE);
    rx_index = 0;
    command_received = false;

    // Start single byte DMA reception (circular mode)
    if (HAL_UART_Receive_DMA(huart, &rx_single_byte, 1) != HAL_OK) {
        return BT_ERROR;
    }
    return BT_OK;
}

// Send JSON data
BT_StatusTypeDef BT_SendJSON(TelemetryData *data) {
    char json_buffer[JSON_BUFFER_SIZE] = {0};
    BT_GenerateJSONString(data, json_buffer);

    uint16_t len = strlen(json_buffer);

    // Check UART state before transmission
    if (huart3.gState != HAL_UART_STATE_READY) {
        printf("[BT] UART not ready, state: %d\r\n", huart3.gState);
        return BT_BUSY;
    }

    printf("[BT] Sending JSON (%d bytes)...\r\n", len);

    // Use longer timeout for large JSON
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart3, (uint8_t*)json_buffer, len, 3000);

    if (status != HAL_OK) {
        printf("[BT] HAL_UART_Transmit failed: %d, ErrorCode: 0x%08lX\r\n", status, huart3.ErrorCode);
        return BT_ERROR;
    }

    printf("[BT] JSON sent successfully\r\n");
    return BT_OK;
}

// Send raw data
BT_StatusTypeDef BT_SendRaw(const char *data, uint16_t size) {
    if (HAL_UART_Transmit(&huart3, (uint8_t*)data, size, 1000) != HAL_OK) {
        return BT_ERROR;
    }
    return BT_OK;
}

void BT_ErrorHandler(void) {
    printf("[ERROR] Reinitializing Bluetooth...\r\n");

    // Stop DMA
    HAL_UART_DMAStop(&huart3);

    // Reset buffers
    memset(rx_buffer, 0, COMMAND_BUFFER_SIZE);
    memset(received_command, 0, COMMAND_BUFFER_SIZE);
    rx_index = 0;
    command_received = false;

    // Restart single byte reception
    HAL_UART_Receive_DMA(&huart3, &rx_single_byte, 1);
}

// Check if command received
bool BT_IsCommandReceived(void) {
    return command_received;
}

// Clear command flag
void BT_ClearCommandFlag(void) {
    command_received = false;
    led_blink_flag = false;
}

// Receive command with timeout
BT_StatusTypeDef BT_ReceiveCommand(char *command, uint32_t timeout) {
    uint32_t start = HAL_GetTick();

    while (!command_received) {
        if (HAL_GetTick() - start > timeout) {
            return BT_TIMEOUT;
        }
        HAL_Delay(1); // Shorter delay
    }

    strncpy(command, received_command, COMMAND_BUFFER_SIZE - 1);
    command[COMMAND_BUFFER_SIZE - 1] = '\0'; // Ensure null termination
    return BT_OK;
}

// Update telemetry data structure
void BT_UpdateTelemetryData(TelemetryData *data) {
    // This function should be implemented to update your telemetry data
    // Example:
    // data->adcs.temperature = readTemperatureSensor();
    // data->eps.battery_percentage = readBatteryPercentage();
    // etc.
}

// Generate JSON string from telemetry data
void BT_GenerateJSONString(TelemetryData *data, char *buffer) {
    // Generate the JSON string based on the telemetry data structure
    snprintf(buffer, JSON_BUFFER_SIZE,
        "{\r\n"
        "  \"timestamp\": \"%s\",\r\n"
        "  \"system_status\": \"%s\",\r\n"
        "  \"adcs\": {\r\n"
        "    \"status\": \"%s\",\r\n"
        "    \"pitch\": %.1f,\r\n"
        "    \"roll\": %.1f,\r\n"
        "    \"yaw\": %.1f,\r\n"
        "    \"gps\": {\r\n"
        "      \"latitude\": %.4f,\r\n"
        "      \"longitude\": %.4f,\r\n"
        "      \"altitude\": %.1f\r\n"
        "    },\r\n"
        "    \"light_sensors\": {\r\n"
        "      \"face_a\": %d,\r\n"
        "      \"face_b\": %d,\r\n"
        "      \"face_c\": %d,\r\n"
        "      \"face_d\": %d\r\n"
        "    },\r\n"
        "    \"temperature\": %d\r\n"
        "  },\r\n"
        "  \"eps\": {\r\n"
        "    \"status\": \"%s\",\r\n"
        "    \"battery_voltage\": %.1f,\r\n"
        "    \"battery_percentage\": %d,\r\n"
        "    \"current\": %.2f,\r\n"
        "    \"solar_power\": %.1f,\r\n"
        "    \"charging_status\": \"%s\",\r\n"
        "    \"boost_output\": %d,\r\n"
        "    \"temperature\": %d\r\n"
        "  },\r\n"
        "  \"obc\": {\r\n"
        "    \"status\": \"%s\",\r\n"
        "    \"data_logging\": \"%s\",\r\n"
        "    \"storage_usage\": %d,\r\n"
        "    \"rtc_sync\": \"%s\",\r\n"
        "    \"i2c_status\": \"%s\",\r\n"
        "    \"command_queue\": %d\r\n"
        "  },\r\n"
        "  \"comms\": {\r\n"
        "    \"status\": \"%s\",\r\n"
        "    \"rf_link\": \"%s\",\r\n"
        "    \"signal_strength\": %d,\r\n"
        "    \"packets_sent\": %d,\r\n"
        "    \"packets_received\": %d,\r\n"
        "    \"last_command\": \"%s\"\r\n"
        "  },\r\n"
        "  \"payload\": {\r\n"
        "    \"status\": \"%s\",\r\n"
        "    \"payload_status\": \"%s\",\r\n"
        "    \"images_today\": %d,\r\n"
        "    \"last_image_size\": %.1f,\r\n"
        "    \"ai_classification\": {\r\n"
        "      \"result\": \"%s\",\r\n"
        "      \"confidence\": %.1f\r\n"
        "    }\r\n"
        "  },\r\n"
        "  \"last_command_button\": \"%s\",\r\n"
        "  \"last_command_time\": \"%s\",\r\n"
        "  \"command_status\": \"%s\"\r\n"
        "}\r\n",
        data->timestamp, data->system_status,
        data->adcs.status, data->adcs.pitch, data->adcs.roll, data->adcs.yaw,
        data->adcs.gps.latitude, data->adcs.gps.longitude, data->adcs.gps.altitude,
        data->adcs.light_sensors.face_a, data->adcs.light_sensors.face_b,
        data->adcs.light_sensors.face_c, data->adcs.light_sensors.face_d,
        data->adcs.temperature,
        data->eps.status, data->eps.battery_voltage, data->eps.battery_percentage,
        data->eps.current, data->eps.solar_power, data->eps.charging_status,
        data->eps.boost_output, data->eps.temperature,
        data->obc.status, data->obc.data_logging, data->obc.storage_usage,
        data->obc.rtc_sync, data->obc.i2c_status, data->obc.command_queue,
        data->comms.status, data->comms.rf_link, data->comms.signal_strength,
        data->comms.packets_sent, data->comms.packets_received, data->comms.last_command,
        data->payload.status, data->payload.payload_status, data->payload.images_today,
        data->payload.last_image_size, data->payload.ai_classification.result,
        data->payload.ai_classification.confidence,
        data->last_command_button, data->last_command_time, data->command_status
    );
}

// Check if LED should blink (call from main loop)
bool BT_ShouldBlinkLED(void) {
    return led_blink_flag;
}

// UART RX Complete callback - handles single byte reception
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        // Process received byte
        if (rx_single_byte == '\n' || rx_single_byte == '\r') {
            // End of command
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0'; // Null terminate

                // Copy to received command buffer
                strncpy(received_command, (char*)rx_buffer, COMMAND_BUFFER_SIZE - 1);
                received_command[COMMAND_BUFFER_SIZE - 1] = '\0';

                printf("[BT] Command received: %s (len: %d)\r\n", received_command, rx_index);

                command_received = true;
                led_blink_flag = true; // Signal main loop to blink LED

                // Reset buffer
                memset(rx_buffer, 0, COMMAND_BUFFER_SIZE);
                rx_index = 0;
            }
        } else if (rx_single_byte >= 32 && rx_single_byte <= 126) { // Printable ASCII
            // Add to buffer if there's space
            if (rx_index < (COMMAND_BUFFER_SIZE - 1)) {
                rx_buffer[rx_index++] = rx_single_byte;
            } else {
                // Buffer overflow protection
                printf("[BT] Buffer overflow, resetting\r\n");
                memset(rx_buffer, 0, COMMAND_BUFFER_SIZE);
                rx_index = 0;
            }
        }
        // Ignore other characters (like control characters)

        // Restart single byte reception
        HAL_UART_Receive_DMA(&huart3, &rx_single_byte, 1);
    }
}

// DMA error callback
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        printf("[BT] UART Error: 0x%08lX\r\n", huart->ErrorCode);

        // Clear error flags
        __HAL_UART_CLEAR_OREFLAG(huart);
        __HAL_UART_CLEAR_NEFLAG(huart);
        __HAL_UART_CLEAR_FEFLAG(huart);
        __HAL_UART_CLEAR_PEFLAG(huart);

        // Restart reception
        BT_ErrorHandler();
    }
}
