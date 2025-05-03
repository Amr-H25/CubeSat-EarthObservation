/*
 * ldr.h
 *
 *  Created on: Apr 11, 2025
 *      Author: Amr_H
 */

#ifndef LIGHT_SENSOR_H
#define LIGHT_SENSOR_H

#include "stm32f1xx_hal.h"
#include "config.h"

// Sensor positions
typedef enum {
    LIGHT_SENSOR_FRONT = 0,
    LIGHT_SENSOR_RIGHT,
    LIGHT_SENSOR_BACK,
    LIGHT_SENSOR_LEFT
} LightSensor_Position;

// Structure to hold all sensor readings
typedef struct {
    uint16_t front;
    uint16_t right;
    uint16_t back;
    uint16_t left;
} LightSensor_Values;

// Function prototypes
void LightSensor_Init(ADC_HandleTypeDef* hadc);
uint16_t LightSensor_ReadSingle(ADC_HandleTypeDef* hadc, LightSensor_Position position);
void LightSensor_ReadAll(ADC_HandleTypeDef* hadc, LightSensor_Values* values);
LightSensor_Position LightSensor_GetMaxLight(LightSensor_Values* values);

#endif /* LIGHT_SENSOR_H */
