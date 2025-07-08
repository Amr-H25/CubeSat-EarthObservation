/*
 * lm35.h
 *
 *  Created on: Apr 11, 2025
 *      Author: Amr_H
 */
#ifndef LM35_H
#define LM35_H

#include "stm32f1xx_hal.h"
#include "config.h"


// Function prototypes
void LM35_Init(ADC_HandleTypeDef* hadc);
float LM35_ReadTemperature(ADC_HandleTypeDef* hadc);

#endif /* LM35_H */
