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
        float pitch;
        float roll;
        float yaw;
        struct {
            float latitude;
            float longitude;
            float altitude;
        } gps;
        struct {
            int face_a;
            int face_b;
            int face_c;
            int face_d;
        } light_sensors;
        int temperature;
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

#endif /* BLUETOOTH_DRIVER_H */
