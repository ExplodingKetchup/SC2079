/*
 * motors.h
 *
 *  Created on: Aug 30, 2023
 *      Author: Duriana
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stm32f4xx_hal.h"

/* The Pins, Ports are as defined below */
#define MOTOR_AIN2_Pin GPIO_PIN_2
#define MOTOR_AIN2_Port GPIOA
#define MOTOR_AIN1_Pin GPIO_PIN_3
#define MOTOR_AIN1_Port GPIOA
#define MOTOR_BIN1_Pin GPIO_PIN_4
#define MOTOR_BIN1_Port GPIOA
#define MOTOR_BIN2_Pin GPIO_PIN_5
#define MOTOR_BIN2_Port GPIOA
#define MOTOR_PWMA_Pin GPIO_PIN_6
#define MOTOR_PWMA_Port GPIOC
#define MOTOR_PWMB_Pin GPIO_PIN_7
#define MOTOR_PWMB_Port GPIOC

/* Directions */
#define DIR_FWD 1
#define DIR_BCK 0

/* Timer used: htim8, channels as defined below */
#define PWMA_TIM_CH TIM_CHANNEL_1
#define PWMB_TIM_CH TIM_CHANNEL_2

/* Limits */
#define MAX_SPEED 7199		// = TIM8.ARR

/* Basic GPIO Set / Clear for GPIO_out Pins */

#define MOTOR_AIN1_Set() HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_SET)	// AIN1 = 1
#define MOTOR_AIN1_Clr() HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_RESET)	// AIN1 = 0

#define MOTOR_AIN2_Set() HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_SET)	// AIN2 = 1
#define MOTOR_AIN2_Clr() HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_RESET) // AIN2 = 0

#define MOTOR_BIN1_Set() HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_SET)	// BIN1 = 1
#define MOTOR_BIN1_Clr() HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_RESET)	// BIN1 = 0

#define MOTOR_BIN2_Set() HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_SET)	// BIN2 = 1
#define MOTOR_BIN2_Clr() HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_RESET) // BIN2 = 0

/* Motor control functions */
void mtr_init(TIM_HandleTypeDef* pwm_htimPtr);
void mtrA_mov(uint8_t direction, uint16_t speed);		// Motor A moves
void mtrB_mov(uint8_t direction, uint16_t speed);		// Motor B moves

#endif /* INC_MOTORS_H_ */
