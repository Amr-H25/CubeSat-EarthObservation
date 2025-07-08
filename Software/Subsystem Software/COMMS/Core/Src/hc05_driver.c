/*
 * hc05-driver.c - Fixed Implementation with State Management
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

// State management variables
static volatile BT_CommunicationState bt_comm_state = BT_COMM_IDLE;
static char* tx_buffer_ptr = NULL;
static uint16_t tx_buffer_size = 0;

// Command queue for commands received during transmission
#define CMD_QUEUE_SIZE 5
static char cmd_queue[CMD_QUEUE_SIZE][COMMAND_BUFFER_SIZE];
static volatile uint8_t cmd_queue_head = 0;
static volatile uint8_t cmd_queue_tail = 0;

extern UART_HandleTypeDef huart2;
#define DEBUG_UART &huart2

// Helper function to queue commands
static bool QueueCommand(const char* command) {
    uint8_t next_head = (cmd_queue_head + 1) % CMD_QUEUE_SIZE;
    if (next_head != cmd_queue_tail) {
        strncpy(cmd_queue[cmd_queue_head], command, COMMAND_BUFFER_SIZE - 1);
        cmd_queue[cmd_queue_head][COMMAND_BUFFER_SIZE - 1] = '\0';
        cmd_queue_head = next_head;
        return true;
    }
    return false; // Queue full
}

// Helper function to dequeue commands
static bool DequeueCommand(char* command) {
    if (cmd_queue_head != cmd_queue_tail) {
        strncpy(command, cmd_queue[cmd_queue_tail], COMMAND_BUFFER_SIZE - 1);
        command[COMMAND_BUFFER_SIZE - 1] = '\0';
        cmd_queue_tail = (cmd_queue_tail + 1) % CMD_QUEUE_SIZE;
        return true;
    }
    return false; // Queue empty
}

// Initialize Bluetooth module
BT_StatusTypeDef BT_Init(UART_HandleTypeDef *huart) {
    // Clear buffers
    memset(rx_buffer, 0, COMMAND_BUFFER_SIZE);
    memset(received_command, 0, COMMAND_BUFFER_SIZE);
    memset(cmd_queue, 0, sizeof(cmd_queue));
    rx_index = 0;
    command_received = false;
    bt_comm_state = BT_COMM_IDLE;
    cmd_queue_head = 0;
    cmd_queue_tail = 0;

    // Start single byte DMA reception (circular mode)
    if (HAL_UART_Receive_DMA(huart, &rx_single_byte, 1) != HAL_OK) {
        return BT_ERROR;
    }
    return BT_OK;
}

// Send JSON data with state management
BT_StatusTypeDef BT_SendJSON(TelemetryData *data) {
    // Check if already transmitting
    if (bt_comm_state != BT_COMM_IDLE) {
        printf("[BT] Bluetooth busy, current state: %d\r\n", bt_comm_state);
        return BT_BUSY;
    }

    // Allocate buffer for JSON (static to persist during DMA)
    static char json_buffer[JSON_BUFFER_SIZE];
    memset(json_buffer, 0, JSON_BUFFER_SIZE);

    BT_GenerateJSONString(data, json_buffer);
    uint16_t len = strlen(json_buffer);

    // Check UART state before transmission
    if (huart3.gState != HAL_UART_STATE_READY) {
        printf("[BT] UART not ready, state: %d\r\n", huart3.gState);
        return BT_BUSY;
    }

    printf("[BT] Sending JSON (%d bytes)...\r\n", len);

    // Set state to transmitting
    bt_comm_state = BT_COMM_TRANSMITTING;
    tx_buffer_ptr = json_buffer;
    tx_buffer_size = len;

    // Use DMA for transmission to get completion callback
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart3, (uint8_t*)json_buffer, len);

    if (status != HAL_OK) {
        printf("[BT] HAL_UART_Transmit_DMA failed: %d, ErrorCode: 0x%08lX\r\n", status, huart3.ErrorCode);
        bt_comm_state = BT_COMM_IDLE; // Reset state on error
        return BT_ERROR;
    }

    return BT_OK;
}

// Send raw data
BT_StatusTypeDef BT_SendRaw(const char *data, uint16_t size) {
    if (bt_comm_state != BT_COMM_IDLE) {
        return BT_BUSY;
    }

    bt_comm_state = BT_COMM_TRANSMITTING;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(&huart3, (uint8_t*)data, size);

    if (status != HAL_OK) {
        bt_comm_state = BT_COMM_IDLE;
        return BT_ERROR;
    }

    return BT_OK;
}

// Get current communication state
BT_CommunicationState BT_GetCommunicationState(void) {
    return bt_comm_state;
}

void BT_ErrorHandler(void) {
    printf("[ERROR] Reinitializing Bluetooth...\r\n");

    // Stop DMA
    HAL_UART_DMAStop(&huart3);

    // Reset state
    bt_comm_state = BT_COMM_IDLE;

    // Reset buffers
    memset(rx_buffer, 0, COMMAND_BUFFER_SIZE);
    memset(received_command, 0, COMMAND_BUFFER_SIZE);
    rx_index = 0;
    command_received = false;

    // Restart single byte reception
    HAL_UART_Receive_DMA(&huart3, &rx_single_byte, 1);
}

// Check if command received (including queued commands)
bool BT_IsCommandReceived(void) {
    return command_received || (cmd_queue_head != cmd_queue_tail);
}

// Clear command flag
void BT_ClearCommandFlag(void) {
    command_received = false;
    led_blink_flag = false;
}

// Receive command with timeout (now handles queued commands)
BT_StatusTypeDef BT_ReceiveCommand(char *command, uint32_t timeout) {
    // First check if there are queued commands
    if (DequeueCommand(command)) {
        printf("[BT] Retrieved queued command: %s\r\n", command);
        return BT_OK;
    }

    // Then check for immediate command
    if (command_received) {
        strncpy(command, received_command, COMMAND_BUFFER_SIZE - 1);
        command[COMMAND_BUFFER_SIZE - 1] = '\0';
        return BT_OK;
    }

    // Wait for new command with timeout
    uint32_t start = HAL_GetTick();
    while (!command_received && (cmd_queue_head == cmd_queue_tail)) {
        if (HAL_GetTick() - start > timeout) {
            return BT_TIMEOUT;
        }
        HAL_Delay(1);
    }

    // Check queued commands first
    if (DequeueCommand(command)) {
        return BT_OK;
    }

    // Then immediate command
    if (command_received) {
        strncpy(command, received_command, COMMAND_BUFFER_SIZE - 1);
        command[COMMAND_BUFFER_SIZE - 1] = '\0';
        return BT_OK;
    }

    return BT_TIMEOUT;
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

// In hc05_driver.c

// In hc05_driver.c

void BT_GenerateJSONString(TelemetryData *data, char *buffer) {
    snprintf(buffer, JSON_BUFFER_SIZE,
        "{"
        "\"timestamp\":\"%s\","
        "\"system_status\":\"%s\","
        "\"adcs\":{"
        "\"status\":\"%s\","
        "\"pitch\":%.1f,"
        "\"roll\":%.1f,"
        "\"yaw\":%.1f,"
        "\"gps\":{\"latitude\":%.4f,\"longitude\":%.4f,\"altitude\":%.1f},"
        "\"light_sensors\":{\"face_a\":%d,\"face_b\":%d,\"face_c\":%d,\"face_d\":%d},"// COMMA ADDED
        "\"temperature\":%d"
        "},"
        "\"eps\":{"
        "\"status\":\"%s\","
        "\"battery_voltage\":%.1f,"
        "\"battery_percentage\":%d,"
        "\"current\":%.2f,"
        "\"solar_power\":%.1f,"
        "\"charging_status\":\"%s\","
        "\"boost_output\":%d,"
        "\"temperature\":%d"
        "},"
        "\"obc\":{"
        "\"status\":\"%s\","
        "\"data_logging\":\"%s\","
        "\"storage_usage\":%d,"
        "\"rtc_sync\":\"%s\","
        "\"i2c_status\":\"%s\","
        "\"command_queue\":%d"
        "},"
        "\"comms\":{"
        "\"status\":\"%s\","
        "\"rf_link\":\"%s\","
        "\"signal_strength\":%d,"
        "\"packets_sent\":%d,"
        "\"packets_received\":%d,"
        "\"last_command\":\"%s\""
        "},"
        "\"payload\":{"
        "\"status\":\"%s\","
        "\"payload_status\":\"%s\","
        "\"images_today\":%d,"
        "\"last_image_size\":%.1f,"
        "\"ai_classification\":{\"result\":\"%s\",\"confidence\":%.1f}"
        "},"
        "\"last_command_button\":\"%s\","
        "\"last_command_time\":\"%s\","
        "\"command_status\":\"%s\""
        "}",
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

// UART TX Complete callback - handles transmission completion
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        printf("[BT] JSON sent successfully\r\n");
        bt_comm_state = BT_COMM_IDLE;

        // Process any queued commands
        if (cmd_queue_head != cmd_queue_tail) {
            printf("[BT] Processing queued commands (%d in queue)\r\n",
                   (cmd_queue_head - cmd_queue_tail + CMD_QUEUE_SIZE) % CMD_QUEUE_SIZE);
            command_received = true;
            led_blink_flag = true;
        }
    }
}

// UART RX Complete callback - handles single byte reception with state management
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART3) {
        // Process received byte
        if (rx_single_byte == '\n' || rx_single_byte == '\r') {
            // End of command
            if (rx_index > 0) {
                rx_buffer[rx_index] = '\0'; // Null terminate

                // Copy to temporary buffer
                char temp_command[COMMAND_BUFFER_SIZE];
                strncpy(temp_command, (char*)rx_buffer, COMMAND_BUFFER_SIZE - 1);
                temp_command[COMMAND_BUFFER_SIZE - 1] = '\0';

                printf("[BT] Command received: %s (len: %d)\r\n", temp_command, rx_index);

                // Handle command based on current state
                if (bt_comm_state == BT_COMM_TRANSMITTING) {
                    // Queue command for later processing
                    if (QueueCommand(temp_command)) {
                        printf("[BT] Command queued during transmission\r\n");
                    } else {
                        printf("[BT] Command queue full! Dropping command\r\n");
                    }
                } else {
                    // Process immediately
                    strncpy(received_command, temp_command, COMMAND_BUFFER_SIZE - 1);
                    received_command[COMMAND_BUFFER_SIZE - 1] = '\0';
                    command_received = true;
                    led_blink_flag = true;
                }

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
                printf("[BT] RX Buffer overflow, resetting\r\n");
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

        // Reset state
        bt_comm_state = BT_COMM_IDLE;

        // Restart reception
        BT_ErrorHandler();
    }
}
