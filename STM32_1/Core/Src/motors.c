/*
 * motors.c
 *
 *  Created on: Aug 30, 2023
 *      Author: Duriana
 */

#include "motors.h"
#include <math.h>
#include <stdlib.h>

MotorData* motorA;
MotorData* motorB;
MotorPIDData* motorAPID;
MotorPIDData* motorBPID;
TIM_HandleTypeDef* htim8Ptr;	// Pointer of the timer for pwm generation (by default should pass &htim8)
TIM_HandleTypeDef* htim2Ptr;	// Pointer of the timer for motor A encoding (by default should pass &htim2)
TIM_HandleTypeDef* htim3Ptr;	// Pointer of the timer for motor B encoding (by default should pass &htim3)

void mtr_init(TIM_HandleTypeDef* pwm_htimPtr, TIM_HandleTypeDef* encodeA_htimPtr, TIM_HandleTypeDef* encodeB_htimPtr,
		MotorData* mtrA, MotorData* mtrB, MotorPIDData* mtrAPID, MotorPIDData* mtrBPID) {

	motorA = mtrA;
	motorB = mtrB;
	motorAPID = mtrAPID;
	motorBPID = mtrBPID;

	mtrA_init(0, 20, 0, 0.001);
	mtrB_init(0, 20, 0, 0.001);

	htim8Ptr = pwm_htimPtr;
	htim2Ptr = encodeA_htimPtr;
	htim3Ptr = encodeB_htimPtr;
	HAL_TIM_PWM_Start(htim8Ptr, PWMA_TIM_CH);
	HAL_TIM_PWM_Start(htim8Ptr, PWMB_TIM_CH);

	HAL_TIM_Encoder_Start_IT(htim2Ptr, TIM_CHANNEL_ALL);	// Note that we only use Channel 1 and 2
	HAL_TIM_Encoder_Start_IT(htim3Ptr, TIM_CHANNEL_ALL);	// Note that we only use Channel 1 and 2
}

void mtrA_init(int16_t target_angle, int16_t Kp, float Kd, float Ki) {

	motorA->dir = DIR_FWD;
	motorA->pwmVal = 0;

	motorAPID->count = 0;       		// Counter (signed value)
	motorAPID->angle = 0;      			// angle of rotation, in degree resolution = 360/260
	motorAPID->target_angle = target_angle; 		// target angle of rotation,
	motorAPID->error = motorAPID->target_angle - motorAPID->angle;           	// error between target and actual
	motorAPID->error_area = 0;  		// area under error - to calculate I for PI implementation
	motorAPID->error_old = 0; 			// to calculate D for PID control
	motorAPID->millisOld = 0;			// to calculate I and D for PID control
	motorAPID->Kp = Kp;
	motorAPID->Kd = Kd;
	motorAPID->Ki = Ki;
}

void mtrB_init(int16_t target_angle, int16_t Kp, float Kd, float Ki) {

	motorB->dir = DIR_FWD;
	motorB->pwmVal = 0;

	motorBPID->count = 0;       		// Counter (signed value)
	motorBPID->angle = 0;      			// angle of rotation, in degree resolution = 360/260
	motorBPID->target_angle = target_angle; 		// target angle of rotation,
	motorBPID->error = motorBPID->target_angle - motorBPID->angle;           	// error between target and actual
	motorBPID->error_area = 0;  		// area under error - to calculate I for PI implementation
	motorBPID->error_old = 0; 			// to calculate D for PID control
	motorBPID->millisOld = 0;			// to calculate I and D for PID control
	motorBPID->Kp = Kp;
	motorBPID->Kd = Kd;
	motorBPID->Ki = Ki;
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

void mtr_mov(MotorData* motor) {
	if (motor == motorA) {
		mtrA_mov(motor->dir, motor->pwmVal);
	}
	else if (motor == motorB) {
		mtrB_mov(motor->dir, motor->pwmVal);
	}
}

void mtr_mov_deg(int degree_A, int degree_B) {
	mtrA_init((int16_t)degree_A,20,0,0.001);
	mtrB_init((int16_t)degree_B,20,0,0.001);
	while ((motorAPID->error > 2) || (motorBPID->error > 2)) {
		PID_Control(motorA, motorAPID);
		PID_Control(motorB, motorBPID);
	}
}

void PID_Control(MotorData* motor, MotorPIDData* motorPID) {
	  //Control Loop
	if (abs(motorPID->error)>2) { //more than 2 degree difference
  	    motorPID->error = motorPID->target_angle - motorPID->angle;

        if (motorPID->error > 0)
        	motor->dir = DIR_FWD;	// Forward
        else
        	motor->dir = DIR_BCK;	// Backward

        int32_t millisNow = HAL_GetTick();
        int32_t dt = (millisNow - motorPID->millisOld); // time elapsed in millisecond
        motorPID->millisOld = millisNow; // store the current time for next round

        motorPID->error_area = motorPID->error_area + motorPID->error * dt; // area under error for Ki

        int32_t error_change = motorPID->error - motorPID->error_old; // change in error
        motorPID->error_old = motorPID->error; //store the error for next round
        float error_rate = (float)error_change / dt; // for Kd

        motor->pwmVal = (int)(motorPID->error * motorPID->Kp + motorPID->error_area * motorPID->Ki + error_rate * motorPID->Kd);  // PID

        mtr_mov(motor);
	} // if loop
}
