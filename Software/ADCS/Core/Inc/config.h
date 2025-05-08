/*
 * config.h
 *
 *  Created on: Apr 24, 2025
 *      Author: Amr_H
 */

/* config.h */
#ifndef CONFIG_H
#define CONFIG_H

#include "stm32f1xx_hal.h"

// L298N Motor Driver Configuration
#define MOTOR_IN1_PORT      GPIOB
#define MOTOR_IN1_PIN       GPIO_PIN_2
#define MOTOR_IN2_PORT      GPIOB
#define MOTOR_IN2_PIN       GPIO_PIN_3
#define MOTOR_IN3_PORT      GPIOB
#define MOTOR_IN3_PIN       GPIO_PIN_4
#define MOTOR_IN4_PORT      GPIOB
#define MOTOR_IN4_PIN       GPIO_PIN_5

// LDR Light Sensor Configuration
#define LDR_FRONT_CHANNEL   ADC_CHANNEL_4
#define LDR_RIGHT_CHANNEL   ADC_CHANNEL_5
#define LDR_BACK_CHANNEL    ADC_CHANNEL_6
#define LDR_LEFT_CHANNEL    ADC_CHANNEL_7
#define LDR_GPIO_PORT       GPIOA
#define LDR_FRONT_PIN       GPIO_PIN_4
#define LDR_RIGHT_PIN       GPIO_PIN_5
#define LDR_BACK_PIN        GPIO_PIN_6
#define LDR_LEFT_PIN        GPIO_PIN_7

// LM35 Temperature Sensor Configuration
#define LM35_ADC_PORT       GPIOB
#define LM35_ADC_PIN        GPIO_PIN_0
#define LM35_ADC_CHANNEL    ADC_CHANNEL_8

#define LED_GPIO_Port		GPIOC
#define LED_Pin				GPIO_PIN_13

#endif /* CONFIG_H */
