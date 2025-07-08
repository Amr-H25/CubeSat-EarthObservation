/*
 * hc05-driver.h
 *
 *  Created on: Jul 3, 2025
 *      Author: Amr_H
 */

/* bluetooth_driver.h */
#ifndef BLUETOOTH_DRIVER_H
#define BLUETOOTH_DRIVER_H

#include "stm32f1xx_hal.h"
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>

#define JSON_BUFFER_SIZE 2048
#define COMMAND_BUFFER_SIZE 64

typedef struct {
    char timestamp[20];
    char system_status[16];

    struct {
        char status[16];
        float pitch;        // From MPU orientation calculations
        float roll;         // From MPU orientation calculations
        float yaw;          // From MPU orientation calculations
        struct {
            float latitude;
            float longitude;
            float altitude;
        } gps;
        struct {
            int face_a;     // LDR front
            int face_b;     // LDR right
            int face_c;     // LDR back
            int face_d;     // LDR left
        } light_sensors;
        int temperature;    // From LM35
        struct {
            float x;
            float y;
            float z;
        } accel;            // From MPU
        struct {
            float x;
            float y;
            float z;
        } gyro;             // From MPU
    } adcs;

    struct {
        char status[16];
        float battery_voltage;
        int battery_percentage;
        float current;
        float solar_power;
        char charging_status[16];
        int boost_output;
        int temperature;
    } eps;

    struct {
        char status[16];
        char data_logging[16];
        int storage_usage;
        char rtc_sync[8];
        char i2c_status[16];
        int command_queue;
    } obc;

    struct {
        char status[16];
        char rf_link[16];
        int signal_strength;
        int packets_sent;
        int packets_received;
        char last_command[16];
    } comms;

    struct {
        char status[16];
        char payload_status[16];
        int images_today;
        float last_image_size;
        struct {
            char result[32];
            float confidence;
        } ai_classification;
    } payload;

    char last_command_button[16];
    char last_command_time[32];
    char command_status[32];
} TelemetryData;

typedef enum {
    BT_OK = 0,
    BT_ERROR,
    BT_BUSY,
    BT_TIMEOUT
} BT_StatusTypeDef;

// New enum for internal state management
typedef enum {
    BT_COMM_IDLE = 0,
    BT_COMM_TRANSMITTING,
    BT_COMM_RECEIVING
} BT_CommunicationState;

// Initialization function
BT_StatusTypeDef BT_Init(UART_HandleTypeDef *huart);

// Data transmission functions
BT_StatusTypeDef BT_SendJSON(TelemetryData *data);
BT_StatusTypeDef BT_SendRaw(const char *data, uint16_t size);

// Data reception functions
BT_StatusTypeDef BT_ReceiveCommand(char *command, uint32_t timeout);
bool BT_IsCommandReceived(void);
void BT_ClearCommandFlag(void);

// Helper functions
void BT_UpdateTelemetryData(TelemetryData *data);
void BT_GenerateJSONString(TelemetryData *data, char *buffer);

bool BT_ShouldBlinkLED(void);

// New function to check communication state
BT_CommunicationState BT_GetCommunicationState(void);

// Error handling
void BT_ErrorHandler(void);

#endif /* BLUETOOTH_DRIVER_H */
