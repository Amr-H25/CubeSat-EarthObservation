/*
 * eps_analog.h
 *
 *  Created on: Jul 4, 2025
 *      Author: Amr_H
 */

#ifndef INC_EPS_ANALOG_H_
#define INC_EPS_ANALOG_H_


#include "stm32f1xx_hal.h"

// Analog channel definitions
typedef enum {
    ANALOG_TEMP = 0,     // PA0 - ADC_CHANNEL_0
    ANALOG_P_X,          // PA1 - ADC_CHANNEL_1
    ANALOG_N_X,          // PA2 - ADC_CHANNEL_2
    ANALOG_P_Y,          // PA3 - ADC_CHANNEL_3
    ANALOG_N_Y,          // PA4 - ADC_CHANNEL_4
    ANALOG_BAT_OUT,      // PA5 - ADC_CHANNEL_5
    ANALOG_CURRENT,      // PA6 - ADC_CHANNEL_6
    ANALOG_CHANNELS_MAX
} analog_channel_t;

// EPS data structure
typedef struct {
    float temperature;
    float solar_p_x;
    float solar_n_x;
    float solar_p_y;
    float solar_n_y;
    float battery_voltage;
    float current_sensor;
} eps_data_t;

// Power control pins
typedef enum {
    POWER_SUBSYS_1 = 0,  // PB12
    POWER_SUBSYS_2,      // PB13
    POWER_SUBSYS_3,      // PB14
    POWER_SUBSYS_MAX
} power_subsys_t;

// Function prototypes
void EPS_Analog_Init(ADC_HandleTypeDef* hadc);
HAL_StatusTypeDef EPS_ReadAllChannels(ADC_HandleTypeDef* hadc, eps_data_t* data);
uint32_t EPS_ReadRawADC(ADC_HandleTypeDef* hadc, uint32_t channel);
float EPS_ConvertToVoltage(uint32_t raw_value);
void EPS_PowerControl(power_subsys_t subsystem, uint8_t enable);
void EPS_PowerControlAll(uint8_t enable);


#endif /* INC_EPS_ANALOG_H_ */
