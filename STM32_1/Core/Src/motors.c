/*
 * motors.c
 *
 *  Created on: Aug 30, 2023
 *      Author: Duriana
 */

#include "motors.h"
#include "servo.h"
#include "oled.h"
#include <math.h>
#include <stdlib.h>

MotorData* motorA;
MotorData* motorB;
MotorPIDData* motorAPID;
MotorPIDData* motorBPID;
TIM_HandleTypeDef* htim8Ptr;	// Pointer of the timer for pwm generation (by default should pass &htim8)
TIM_HandleTypeDef* htim2Ptr;	// Pointer of the timer for motor A encoding (by default should pass &htim2)
TIM_HandleTypeDef* htim3Ptr;	// Pointer of the timer for motor B encoding (by default should pass &htim3)
osSemaphoreId_t* ori_semaphoreHandlePtr;

void mtr_init(TIM_HandleTypeDef* pwm_htimPtr, TIM_HandleTypeDef* encodeA_htimPtr, TIM_HandleTypeDef* encodeB_htimPtr,
		MotorData* mtrA, MotorData* mtrB, MotorPIDData* mtrAPID, MotorPIDData* mtrBPID, osSemaphoreId_t* oriSemHandlePtr) {

	motorA = mtrA;
	motorB = mtrB;
	motorAPID = mtrAPID;
	motorBPID = mtrBPID;

	ori_semaphoreHandlePtr = oriSemHandlePtr;

	htim8Ptr = pwm_htimPtr;
	htim2Ptr = encodeA_htimPtr;
	htim3Ptr = encodeB_htimPtr;
	HAL_TIM_PWM_Start(htim8Ptr, PWMA_TIM_CH);
	HAL_TIM_PWM_Start(htim8Ptr, PWMB_TIM_CH);

	HAL_TIM_Encoder_Start_IT(htim2Ptr, TIM_CHANNEL_ALL);	// Note that we only use Channel 1 and 2
	HAL_TIM_Encoder_Start_IT(htim3Ptr, TIM_CHANNEL_ALL);	// Note that we only use Channel 1 and 2

	mtrA_init(0, 0, 0, 0, 1);
	mtrB_init(0, 0, 0, 0, 1);
}

void mtrA_init(int16_t target, int16_t Kp, float Kd, float Ki, uint8_t reset_timer) {

	if (reset_timer)
		__HAL_TIM_SET_COUNTER(htim2Ptr, 0);

	motorA->dir = DIR_FWD;
	motorA->pwmVal = 0;

	motorAPID->count = -(int16_t)__HAL_TIM_GET_COUNTER(htim2Ptr);       		// Counter (signed value)
	//motorAPID->angle = 0;      			// angle of rotation, in degree resolution = 360/260
	//motorAPID->target_angle = target_angle; 		// target angle of rotation,
	motorAPID->target = target;
	//motorAPID->error = motorAPID->target_angle - motorAPID->angle;           	// error between target and actual
	motorAPID->error = motorAPID->target - motorAPID->count;
	motorAPID->error_area = 0;  		// area under error - to calculate I for PI implementation
	motorAPID->error_old = 0; 			// to calculate D for PID control
	motorAPID->millisOld = HAL_GetTick();			// to calculate I and D for PID control
	motorAPID->Kp = Kp;
	motorAPID->Kd = Kd;
	motorAPID->Ki = Ki;
}

void mtrB_init(int16_t target, int16_t Kp, float Kd, float Ki, uint8_t reset_timer) {

	if (reset_timer)
		__HAL_TIM_SET_COUNTER(htim3Ptr, 0);

	motorB->dir = DIR_FWD;
	motorB->pwmVal = 0;

	motorBPID->count = (int16_t)__HAL_TIM_GET_COUNTER(htim3Ptr);       		// Counter (signed value)
	//motorBPID->angle = 0;      			// angle of rotation, in degree resolution = 360/260
	//motorBPID->target_angle = target_angle; 		// target angle of rotation,
	motorBPID->target = target;
	//motorBPID->error = motorBPID->target_angle - motorBPID->angle;           	// error between target and actual
	motorBPID->error = motorBPID->target - motorBPID->count;
	motorBPID->error_area = 0;  		// area under error - to calculate I for PI implementation
	motorBPID->error_old = 0; 			// to calculate D for PID control
	motorBPID->millisOld = HAL_GetTick();			// to calculate I and D for PID control
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

void mtr_stop() {
	mtrA_mov(DIR_FWD, 0);
	mtrB_mov(DIR_FWD, 0);
	turnServo(STRAIGHT);
}

void mtr_mov_cnt(int target_A, int target_B) {
	mtrA_init((int16_t)target_A, 1.2, 0.05, 0.0001, 1);
	mtrB_init((int16_t)target_B, 1.2, 0.05, 0.0001, 1);
	while ((abs(motorAPID->error) > MAX_PID_ERR) || (abs(motorBPID->error) > MAX_PID_ERR)) {
		PID_Control(motorA, motorAPID);
		PID_Control(motorB, motorBPID);
	}
	mtr_stop();
	osDelay(700);
	mtrA_init((int16_t)target_A, 0.3, 0, 0.0001, 0);
	mtrB_init((int16_t)target_B, 0.3, 0, 0.0001, 0);
	while ((abs(motorAPID->error) > MAX_PID_ERR) || (abs(motorBPID->error) > MAX_PID_ERR)) {
		PID_Control(motorA, motorAPID);
		PID_Control(motorB, motorBPID);
	}
	mtr_stop();
	osDelay(500);
	mtrA_init((int16_t)target_A, 0.1, 0, 0.0001, 0);
	mtrB_init((int16_t)target_B, 0.1, 0, 0.0001, 0);
	while ((abs(motorAPID->error) > MAX_PID_ERR) || (abs(motorBPID->error) > MAX_PID_ERR)) {
		PID_Control(motorA, motorAPID);
		PID_Control(motorB, motorBPID);
	}
	mtr_stop();
}

void mtr_mov_cm(float cm_A, float cm_B) {
	mtr_mov_cnt((int)(cm_A * CNT_PER_CM), (int)(cm_B * CNT_PER_CM));
}

/*void PID_Control(MotorData* motor, MotorPIDData* motorPID) {
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
}*/
void PID_Control(MotorData* motor, MotorPIDData* motorPID) {
	  //Control Loop
	if (abs(motorPID->error)>MAX_PID_ERR) { //more than 100  difference
  	    motorPID->error = motorPID->target - motorPID->count;

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

        motor->pwmVal = abs((int32_t)(motorPID->error * motorPID->Kp + motorPID->error_area * motorPID->Ki + error_rate * motorPID->Kd)) / 2;  // PID
        osDelay(10);
        if (motor->pwmVal > MAX_SPEED)
        	motor->pwmVal = MAX_SPEED;
        if (motor->pwmVal < MIN_SPEED)
        	motor-> pwmVal = MIN_SPEED;

        mtr_mov(motor);
	} // if loop
	else {
		motor->dir = DIR_FWD;
		motor->pwmVal = 0;
		mtr_mov(motor);
	}
}

/*
void turn(float target_ori, float* orientation) {
	//osSemaphoreWait(*ori_semaphoreHandlePtr, osWaitForever);
	float turning_angle = target_ori - (*orientation);
	//osSemaphoreRelease(*ori_semaphoreHandlePtr);
	if (abs(turning_angle) < MAX_ORI_ERR)		// Too small, turn may not be accurate
		return;
	if (turning_angle < 0)
		turning_angle += 360;

	uint8_t near_0 = 0;
	if ((target_ori < MAX_ORI_ERR) || (target_ori > 360 - MAX_ORI_ERR)) {
		near_0 = 1;
	}

	if (turning_angle <= 180) {	// Turn left
		turnServo(LEFT);
		mtrA_mov(DIR_BCK, 1800);
		mtrB_mov(DIR_FWD, 1800);
	}
	else {
		turnServo(RIGHT);
		mtrA_mov(DIR_FWD, 1800);
		mtrB_mov(DIR_BCK, 2000);
	}
	if (!near_0) {
		while (abs((*orientation) - target_ori) > MAX_ORI_ERR) {
			osDelay(5);
		}
	}
	else {
		float bound_lo, bound_hi;
		if (target_ori > 350) {
			bound_lo = target_ori - MAX_ORI_ERR;
			bound_hi = target_ori + MAX_ORI_ERR - 360;
		}
		else {
			bound_lo = target_ori - MAX_ORI_ERR + 360;
			bound_hi = target_ori + MAX_ORI_ERR;
		}
		while (((*orientation) < bound_lo) && ((*orientation) > bound_hi)) {
			osDelay(5);
		}
	}
	mtr_stop();
}*/

void turn(float target_ori, float* orientation) {
	//osSemaphoreWait(*ori_semaphoreHandlePtr, osWaitForever);
	float turning_angle = target_ori - (*orientation);
	//osSemaphoreRelease(*ori_semaphoreHandlePtr);
	if (abs(turning_angle) < MAX_ORI_ERR)		// Too small, turn may not be accurate
		return;
	if (turning_angle < 0)
		turning_angle += 360;

	uint8_t near_0 = 0;
	if ((target_ori < MAX_ORI_ERR) || (target_ori > 360 - MAX_ORI_ERR)) {
		near_0 = 1;
	}

	uint8_t mtr_dir = 1;	// 0: stop, 1: fwd, 2: bck

	while (mtr_dir) {
		if (turning_angle <= 180) {	// Turn left
			if (mtr_dir == 1) {
				turnServo(LEFT);
				mtrA_mov(DIR_FWD, 1200);
				mtrB_mov(DIR_FWD, 1200);
				mtr_dir = 2;
			}
			else {
				turnServo(RIGHT);
				mtrA_mov(DIR_BCK, 1200);
				mtrB_mov(DIR_BCK, 1200);
				mtr_dir = 1;
			}
		}
		else {
			if (mtr_dir == 1) {
				turnServo(RIGHT);
				mtrA_mov(DIR_FWD, 1200);
				mtrB_mov(DIR_FWD, 1200);
				mtr_dir = 2;
			}
			else {
				turnServo(LEFT);
				mtrA_mov(DIR_BCK, 1200);
				mtrB_mov(DIR_BCK, 1200);
				mtr_dir = 1;
			}
		}
		if (!near_0) {
			for (int i = 0; i < 100; i++) {
				if (abs((*orientation) - target_ori) < MAX_ORI_ERR) {
					mtr_dir = 0;
					break;
				}
				osDelay(5);
			}
		}
		else {
			float bound_lo, bound_hi;
			if (target_ori > 350) {
				bound_lo = target_ori - MAX_ORI_ERR;
				bound_hi = target_ori + MAX_ORI_ERR - 360;
			}
			else {
				bound_lo = target_ori - MAX_ORI_ERR + 360;
				bound_hi = target_ori + MAX_ORI_ERR;
			}
			for (int i = 0; i < 100; i++) {
				if (((*orientation) > bound_lo) || ((*orientation) < bound_hi)) {
					mtr_dir = 0;
					break;
				}
				osDelay(5);
			}
		}
		mtr_stop();
	}
	mtr_stop();
}
