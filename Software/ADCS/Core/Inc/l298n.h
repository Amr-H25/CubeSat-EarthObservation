/*
 * l298n.h
 *
 *  Created on: Apr 11, 2025
 *      Author: Amr_H
 */

#ifndef L298N_H
#define L298N_H

#include "stm32f1xx_hal.h"
#include "config.h"

// Operation modes
typedef enum {
    L298N_SINGLE_CHANNEL,
    L298N_DUAL_CHANNEL
} L298N_Mode;

// Motor direction
typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_STOP
} Motor_Direction;

// Motor channel
typedef enum {
    MOTOR_A,
    MOTOR_B
} Motor_Channel;

// Add initialization structure
typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel_a;
    uint32_t channel_b;
    L298N_Mode mode;
} L298N_InitTypeDef;

typedef struct {
    TIM_HandleTypeDef* htim;
    uint32_t channel_a;
    uint32_t channel_b;
    GPIO_TypeDef* in1_port;
    uint16_t in1_pin;
    GPIO_TypeDef* in2_port;
    uint16_t in2_pin;
    GPIO_TypeDef* in3_port;
    uint16_t in3_pin;
    GPIO_TypeDef* in4_port;
    uint16_t in4_pin;
    L298N_Mode mode;
} L298N_HandleTypeDef;

void L298N_Init(L298N_HandleTypeDef* hl298n, L298N_InitTypeDef* init);
void L298N_SetSpeed(L298N_HandleTypeDef* hl298n, Motor_Channel channel, uint8_t speed);
void L298N_SetDirection(L298N_HandleTypeDef* hl298n, Motor_Channel channel, Motor_Direction direction);
void L298N_StopMotor(L298N_HandleTypeDef* hl298n, Motor_Channel channel);

#endif /* L298N_H */
