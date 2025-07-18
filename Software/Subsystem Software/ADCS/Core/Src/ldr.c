/*
 * ldr.c
 *
 *  Created on: Apr 11, 2025
 *      Author: Amr_H
 */


#include "ldr.h"

// ADC channel mapping remains the same
static const uint32_t SENSOR_CHANNELS[] = {
    LDR_FRONT_CHANNEL,
    LDR_RIGHT_CHANNEL,
    LDR_BACK_CHANNEL,
    LDR_LEFT_CHANNEL
};

void LightSensor_Init(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // Enable GPIO Clock
    __HAL_RCC_GPIOA_CLK_ENABLE();

    // Configure GPIO pins for analog input using config.h definitions
    GPIO_InitStruct.Pin = LDR_FRONT_PIN | LDR_RIGHT_PIN | LDR_BACK_PIN | LDR_LEFT_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LDR_GPIO_PORT, &GPIO_InitStruct);
}

uint16_t LightSensor_ReadSingle(ADC_HandleTypeDef* hadc, LightSensor_Position position)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint16_t adcValue = 0;

    // Configure ADC channel
    sConfig.Channel = SENSOR_CHANNELS[position];
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;

    // Configure channel and make sure to reset any previous configuration
    HAL_ADC_Stop(hadc);
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    // Start ADC conversion
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 100);
    adcValue = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    return adcValue;
}

void LightSensor_ReadAll(ADC_HandleTypeDef* hadc, LightSensor_Values* values)
{
    values->front = LightSensor_ReadSingle(hadc, LIGHT_SENSOR_FRONT);
    values->right = LightSensor_ReadSingle(hadc, LIGHT_SENSOR_RIGHT);
    values->back = LightSensor_ReadSingle(hadc, LIGHT_SENSOR_BACK);
    values->left = LightSensor_ReadSingle(hadc, LIGHT_SENSOR_LEFT);
}

LightSensor_Position LightSensor_GetMaxLight(LightSensor_Values* values)
{
    uint16_t max_value = values->front;
    LightSensor_Position max_position = LIGHT_SENSOR_FRONT;

    if(values->right > max_value) {
        max_value = values->right;
        max_position = LIGHT_SENSOR_RIGHT;
    }
    if(values->back > max_value) {
        max_value = values->back;
        max_position = LIGHT_SENSOR_BACK;
    }
    if(values->left > max_value) {
        max_value = values->left;
        max_position = LIGHT_SENSOR_LEFT;
    }

    return max_position;
}
