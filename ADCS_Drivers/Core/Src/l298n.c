/*
 * l298n.c
 *
 *  Created on: Apr 11, 2025
 *      Author: Amr_H
 */


#include "l298n.h"

void L298N_Init(L298N_HandleTypeDef* hl298n)
{
    // Start PWM
    HAL_TIM_PWM_Start(hl298n->htim, hl298n->channel_a);
    if(hl298n->mode == L298N_DUAL_CHANNEL) {
        HAL_TIM_PWM_Start(hl298n->htim, hl298n->channel_b);
    }

    // Ensure motors are stopped
    L298N_StopMotor(hl298n, MOTOR_A);
    if(hl298n->mode == L298N_DUAL_CHANNEL) {
        L298N_StopMotor(hl298n, MOTOR_B);
    }
}

void L298N_SetSpeed(L298N_HandleTypeDef* hl298n, Motor_Channel channel, uint8_t speed)
{
    uint32_t pwm_value = (speed * 999) / 100;  // Convert percentage to PWM value

    if(channel == MOTOR_A) {
        __HAL_TIM_SET_COMPARE(hl298n->htim, hl298n->channel_a, pwm_value);
    }
    else if(channel == MOTOR_B && hl298n->mode == L298N_DUAL_CHANNEL) {
        __HAL_TIM_SET_COMPARE(hl298n->htim, hl298n->channel_b, pwm_value);
    }
}

void L298N_SetDirection(L298N_HandleTypeDef* hl298n, Motor_Channel channel, Motor_Direction direction)
{
	if(channel == MOTOR_A) {
	        // First, stop the motor
	        HAL_GPIO_WritePin(hl298n->in1_port, hl298n->in1_pin, GPIO_PIN_RESET);
	        HAL_GPIO_WritePin(hl298n->in2_port, hl298n->in2_pin, GPIO_PIN_RESET);
	        HAL_Delay(10);  // Short delay

	        // Then set the new direction
	        switch(direction) {
	            case MOTOR_FORWARD:
	                HAL_GPIO_WritePin(hl298n->in1_port, hl298n->in1_pin, GPIO_PIN_SET);
	                HAL_GPIO_WritePin(hl298n->in2_port, hl298n->in2_pin, GPIO_PIN_RESET);
	                break;
	            case MOTOR_BACKWARD:
	                HAL_GPIO_WritePin(hl298n->in1_port, hl298n->in1_pin, GPIO_PIN_RESET);
	                HAL_GPIO_WritePin(hl298n->in2_port, hl298n->in2_pin, GPIO_PIN_SET);
	                break;
	            case MOTOR_STOP:
	                HAL_GPIO_WritePin(hl298n->in1_port, hl298n->in1_pin, GPIO_PIN_RESET);
	                HAL_GPIO_WritePin(hl298n->in2_port, hl298n->in2_pin, GPIO_PIN_RESET);
	                break;
	        }
	    }
    else if(channel == MOTOR_B && hl298n->mode == L298N_DUAL_CHANNEL) {
        // Stop motor before changing direction
        HAL_GPIO_WritePin(hl298n->in3_port, hl298n->in3_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(hl298n->in4_port, hl298n->in4_pin, GPIO_PIN_RESET);
        HAL_Delay(10);  // Short delay for safety

        switch(direction) {
            case MOTOR_FORWARD:
                HAL_GPIO_WritePin(hl298n->in3_port, hl298n->in3_pin, GPIO_PIN_SET);
                HAL_GPIO_WritePin(hl298n->in4_port, hl298n->in4_pin, GPIO_PIN_RESET);
                break;
            case MOTOR_BACKWARD:
                HAL_GPIO_WritePin(hl298n->in3_port, hl298n->in3_pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(hl298n->in4_port, hl298n->in4_pin, GPIO_PIN_SET);
                break;
            case MOTOR_STOP:
                HAL_GPIO_WritePin(hl298n->in3_port, hl298n->in3_pin, GPIO_PIN_RESET);
                HAL_GPIO_WritePin(hl298n->in4_port, hl298n->in4_pin, GPIO_PIN_RESET);
                break;
        }
    }
}

void L298N_StopMotor(L298N_HandleTypeDef* hl298n, Motor_Channel channel)
{
    if(channel == MOTOR_A) {
        L298N_SetSpeed(hl298n, MOTOR_A, 0);
        HAL_GPIO_WritePin(hl298n->in1_port, hl298n->in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(hl298n->in2_port, hl298n->in2_pin, GPIO_PIN_RESET);
    }
    else if(channel == MOTOR_B && hl298n->mode == L298N_DUAL_CHANNEL) {
        L298N_SetSpeed(hl298n, MOTOR_B, 0);
        HAL_GPIO_WritePin(hl298n->in3_port, hl298n->in3_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(hl298n->in4_port, hl298n->in4_pin, GPIO_PIN_RESET);
    }
}
