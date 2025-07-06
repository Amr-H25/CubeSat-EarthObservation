/*
 * lm35.c
 *
 *  Created on: Apr 11, 2025
 *      Author: Amr_H
 */
#include "lm35.h"

void LM35_Init(ADC_HandleTypeDef* hadc)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = LM35_ADC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(LM35_ADC_PORT, &GPIO_InitStruct);
}

float LM35_ReadTemperature(ADC_HandleTypeDef* hadc)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t adcValue = 0;
    float temperature;

    // First stop any ongoing conversion
    HAL_ADC_Stop(hadc);

    // Configure for LM35 channel
    sConfig.Channel = LM35_ADC_CHANNEL;
    sConfig.Rank = ADC_REGULAR_RANK_1;
    sConfig.SamplingTime = ADC_SAMPLETIME_55CYCLES_5;
    HAL_ADC_ConfigChannel(hadc, &sConfig);

    // Now read the value
    HAL_ADC_Start(hadc);
    HAL_ADC_PollForConversion(hadc, 100);
    adcValue = HAL_ADC_GetValue(hadc);
    HAL_ADC_Stop(hadc);

    temperature = ((float)adcValue * 3.3f * 100.0f) / 4096.0f;

    // Compensate for offset
    temperature -= 0.22f;  // Subtract the observed offset

    // Ensure temperature doesn't go negative due to offset correction
    if(temperature < 0)
        temperature = 0;

    return temperature;
}

