/*
 * eps_analog.c
 *
 *  Created on: Jul 4, 2025
 *      Author: Amr_H
 */



#include "eps_analog.h"
#include "lm35.h"
#include "main.h"

// ADC channel mapping
static const uint32_t adc_channels[ANALOG_CHANNELS_MAX] = {
    ADC_CHANNEL_0,  // Temperature
    ADC_CHANNEL_1,  // Solar P_X
    ADC_CHANNEL_2,  // Solar N_X
    ADC_CHANNEL_3,  // Solar P_Y
    ADC_CHANNEL_4,  // Solar N_Y
    ADC_CHANNEL_5,  // Battery output
    ADC_CHANNEL_6   // Current sensor
};

// Power control pin mapping
static const uint16_t power_pins[POWER_SUBSYS_MAX] = {
    GPIO_PIN_12,  // PB12
    GPIO_PIN_13,  // PB13
    GPIO_PIN_14   // PB14
};

void EPS_Analog_Init(ADC_HandleTypeDef* hadc)
{
    // ADC is already initialized in main.c
    // Just ensure ADC is ready for use
    HAL_ADC_Start(hadc);
    HAL_ADC_Stop(hadc);
}

HAL_StatusTypeDef EPS_ReadAllChannels(ADC_HandleTypeDef* hadc, eps_data_t* data)
{
    if (data == NULL) {
        return HAL_ERROR;
    }

    // Read temperature using LM35 driver
    data->temperature = LM35_ReadTemperature(hadc);

    // Read solar panel voltages
    data->solar_p_x = EPS_ConvertToVoltage(EPS_ReadRawADC(hadc, ADC_CHANNEL_1));
    data->solar_n_x = EPS_ConvertToVoltage(EPS_ReadRawADC(hadc, ADC_CHANNEL_2));
    data->solar_p_y = EPS_ConvertToVoltage(EPS_ReadRawADC(hadc, ADC_CHANNEL_3));
    data->solar_n_y = EPS_ConvertToVoltage(EPS_ReadRawADC(hadc, ADC_CHANNEL_4));

    // Read battery voltage
    data->battery_voltage = EPS_ConvertToVoltage(EPS_ReadRawADC(hadc, ADC_CHANNEL_5));

    // Read current sensor (may need scaling based on your sensor)
    data->current_sensor = EPS_ConvertToVoltage(EPS_ReadRawADC(hadc, ADC_CHANNEL_6));

    return HAL_OK;
}

uint32_t EPS_ReadRawADC(ADC_HandleTypeDef* hadc, uint32_t channel)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t adc_value = 0;

    // Stop any ongoing conversion
    HAL_ADC_Stop(hadc);

    // Configure the channel
    sConfig.Channel = channel;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    // Start conversion and read value
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 100);
    adc_value = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    return adc_value;
}

float EPS_ConvertToVoltage(uint32_t raw_value)
{
    // Convert 12-bit ADC value to voltage (0-3.3V)
    return ((float)raw_value * 3.3f) / 4096.0f;
}

void EPS_PowerControl(power_subsys_t subsystem, uint8_t enable)
{
    if (subsystem >= POWER_SUBSYS_MAX) {
        return;
    }

    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;
    HAL_GPIO_WritePin(GPIOB, power_pins[subsystem], state);
}

void EPS_PowerControlAll(uint8_t enable)
{
    GPIO_PinState state = enable ? GPIO_PIN_SET : GPIO_PIN_RESET;

    for (int i = 0; i < POWER_SUBSYS_MAX; i++) {
        HAL_GPIO_WritePin(GPIOB, power_pins[i], state);
    }
}
