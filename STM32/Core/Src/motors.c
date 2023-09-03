/*
 * motors.c
 *
 *  Created on: Aug 30, 2023
 *      Author: Duriana
 */

#include "motors.h"

TIM_HandleTypeDef* htim8Ptr;	// Pointer of the timer for pwm generation (by default should pass &htim8)

void mtr_init(TIM_HandleTypeDef* pwm_htimPtr) {
	htim8Ptr = pwm_htimPtr;
	HAL_TIM_PWM_Start(htim8Ptr, PWMA_TIM_CH);
	HAL_TIM_PWM_Start(htim8Ptr, PWMB_TIM_CH);
}

void mtrA_mov(uint8_t direction, uint16_t speed) {

	// Backward
	if (direction == DIR_BCK) {
		MOTOR_AIN1_Clr();
		MOTOR_AIN2_Set();
	}
	// Forward
	else {
		MOTOR_AIN1_Set();
		MOTOR_AIN2_Clr();
	}

	// Set speed
	if (speed > MAX_SPEED) speed = MAX_SPEED;
	__HAL_TIM_SET_COMPARE(htim8Ptr, PWMA_TIM_CH, speed);
}

void mtrB_mov(uint8_t direction, uint16_t speed) {

	// Backward
	if (direction == DIR_BCK) {
		MOTOR_BIN1_Clr();
		MOTOR_BIN2_Set();
	}
	// Forward
	else {
		MOTOR_BIN1_Set();
		MOTOR_BIN2_Clr();
	}

	// Set speed
	if (speed > MAX_SPEED) speed = MAX_SPEED;
	__HAL_TIM_SET_COMPARE(htim8Ptr, PWMB_TIM_CH, speed);
}
