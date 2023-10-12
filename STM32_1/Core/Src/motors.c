/*
 * motors.c
 *
 *  Created on: Aug 30, 2023
 *      Author: Duriana
 */

#include "motors.h"
#include "main.h"
#include "servo.h"
#include "oled.h"
#include <math.h>
#include <stdlib.h>

MotorData* motorA;
MotorData* motorB;
MotorPIDData* motorAPID;
MotorPIDData* motorBPID;
MotorServoStatus* backup;
float* ori;
TIM_HandleTypeDef* htim8Ptr;	// Pointer of the timer for pwm generation (by default should pass &htim8)
TIM_HandleTypeDef* htim2Ptr;	// Pointer of the timer for motor A encoding (by default should pass &htim2)
TIM_HandleTypeDef* htim3Ptr;	// Pointer of the timer for motor B encoding (by default should pass &htim3)
osSemaphoreId_t* ori_semaphoreHandlePtr;


/* Private function prototypes */
void backup_reset();

/* All functions */

void mtr_init(TIM_HandleTypeDef* pwm_htimPtr, TIM_HandleTypeDef* encodeA_htimPtr, TIM_HandleTypeDef* encodeB_htimPtr,
		MotorData* mtrA, MotorData* mtrB, MotorPIDData* mtrAPID, MotorPIDData* mtrBPID, MotorServoStatus* backupObj,
		float* orientation, osSemaphoreId_t* oriSemHandlePtr) {

	motorA = mtrA;
	motorB = mtrB;
	motorA->suspend = 0;
	motorA->suspend = 0;

	motorAPID = mtrAPID;
	motorBPID = mtrBPID;

	backup = backupObj;
	backup_reset();

	ori = orientation;
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

void backup_reset() {
	backup->servoDir = STRAIGHT;
	backup->mtrADir = DIR_FWD;
	backup->mtrAPWM = 0;
	backup->mtrBDir = DIR_FWD;
	backup->mtrBPWM = 0;
}

void mtrA_init(int16_t target, int16_t Kp, float Kd, float Ki, uint8_t reset_timer) {

	if (reset_timer)
		__HAL_TIM_SET_COUNTER(htim2Ptr, 0);

	motorA->dir = DIR_FWD;
	motorA->pwmVal = 0;

	motorAPID->count = -(int16_t)__HAL_TIM_GET_COUNTER(htim2Ptr);       		// Counter (signed value)
	motorAPID->target = target;
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
	motorBPID->target = target;
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

/* Set params and stop both motors */
void mtr_stop() {
	motorA->dir = DIR_FWD;
	motorA->pwmVal = 0;
	motorB->dir = DIR_FWD;
	motorB->pwmVal = 0;
	mtrA_mov(DIR_FWD, 0);
	mtrB_mov(DIR_FWD, 0);
	turnServo(STRAIGHT);
	osDelay(100);
}

void mtr_mov(MotorData* motor) {
	if (motor->suspend > 0) {
		mtr_stop();
		mtr_continue();
		return;
	}
	if (motor == motorA)  {
		mtrA_mov(motor->dir, motor->pwmVal);
	}
	else if (motor == motorB) {
		mtrB_mov(motor->dir, motor->pwmVal);
	}
}

void mtr_SetParamAndMove(MotorData* motor, uint8_t param_dir, uint32_t param_pwmVal) {
	motor->dir = param_dir;
	motor->pwmVal = param_pwmVal;
	mtr_mov(motor);
}

void mtr_suspend(uint8_t mode) {
	if (mode > 3) return;
	if ((motorA->suspend != SUS_OFF) || (motorB->suspend != SUS_OFF)) return;	// Suspend is in effect / being resolved
	HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_RESET);
	motorA->suspend = mode;
	motorB->suspend = mode;
	if ((mode == SUS_BACK) || (mode == SUS_STOP)) {
		backup->servoDir = getServoDir();
		backup->mtrADir = motorA->dir;
		backup->mtrAPWM = motorA->pwmVal;
		backup->mtrBDir = motorB->dir;
		backup->mtrBPWM = motorB->pwmVal;
	}
	mtr_stop();
}

/*
 * Restore movement, is called in mtr_mov() if the suspend status is on
 */
void mtr_continue() {
	if (!motorA->suspend) return;	// Not suspended or likely an error

	if ((motorA->suspend == SUS_BACK) || (motorA->suspend == SUS_STOPPID)) {	// Needs SOSBack
		mtr_SOSBack();
		//HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_RESET);
	}
	if (motorA->suspend == SUS_STOPPID) {	// Stop PID, do not restore movement
		stopPID();
	}
	else {									// Restore movement, in case of SUS_BACK
		turnServo(backup->servoDir);
		motorA->dir = backup->mtrADir;
		motorA->pwmVal = backup->mtrAPWM;
		motorB->dir = backup->mtrBDir;
		motorB->pwmVal = backup->mtrBPWM;
		mtrA_mov(motorA->dir, motorA->pwmVal);
		mtrB_mov(motorB->dir, motorB->pwmVal);
	}
	backup_reset();
	motorA->suspend = 0;
	motorB->suspend = 0;
	HAL_GPIO_WritePin(GPIOE, LED3_Pin, GPIO_PIN_SET);
}

/* Used in emergency cases when car is too close to obstacles only */
float mtr_SOSBack() {
	mtrA_mov(DIR_BCK, 1800);
	mtrB_mov(DIR_BCK, 1800);
	osDelay(500);
	mtr_stop();
	return SOSBACK_DIST_CNT / CNT_PER_CM;
}

void mtr_mov_cnt(int target_A, int target_B) {
	mtrA_init((int16_t)target_A, 1.2, 0.05, 0.0001, 1);
	mtrB_init((int16_t)target_B, 1.2, 0.05, 0.0001, 1);
	while ((abs(motorAPID->error) > MAX_PID_ERR) || (abs(motorBPID->error) > MAX_PID_ERR)) {
		PID_Control(motorA, motorAPID);
		PID_Control(motorB, motorBPID);
		mtr_mov(motorA);
		mtr_mov(motorB);
	}
	mtr_stop();
}

void mtr_mov_cnt_line(int target_A, int target_B) {
	mtrA_init((int16_t)target_A, 1.2, 0.05, 0.0001, 1);
	mtrB_init((int16_t)target_B, 1.2, 0.05, 0.0001, 1);
	while ((abs(motorAPID->error) > MAX_PID_ERR) || (abs(motorBPID->error) > MAX_PID_ERR)) {
		PID_Control(motorA, motorAPID);
		PID_Control(motorB, motorBPID);
        mtr_mov(motorA);
        mtr_mov(motorB);
	}
	mtr_stop();
	osDelay(700);
	mtrA_init((int16_t)target_A, 0.3, 0, 0.0001, 0);
	mtrB_init((int16_t)target_B, 0.3, 0, 0.0001, 0);
	while ((abs(motorAPID->error) > MAX_PID_ERR) || (abs(motorBPID->error) > MAX_PID_ERR)) {
		PID_Control(motorA, motorAPID);
		PID_Control(motorB, motorBPID);
		mtr_mov(motorA);
		mtr_mov(motorB);
	}
	mtr_stop();
	osDelay(500);
	mtrA_init((int16_t)target_A, 0.1, 0, 0.0001, 0);
	mtrB_init((int16_t)target_B, 0.1, 0, 0.0001, 0);
	while ((abs(motorAPID->error) > MAX_PID_ERR) || (abs(motorBPID->error) > MAX_PID_ERR)) {
		PID_Control(motorA, motorAPID);
		PID_Control(motorB, motorBPID);
		mtr_mov(motorA);
		mtr_mov(motorB);
	}
	mtr_stop();
}


void mtr_mov_cm(float cm_A, float cm_B) {
	mtr_mov_cnt((int)(cm_A * CNT_PER_CM), (int)(cm_B * CNT_PER_CM));
}

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
	} // if loop
	else {
		motor->dir = DIR_FWD;
		motor->pwmVal = 0;
	}
}

void stopPID() {
	motorAPID->error = 0;
	motorBPID->error = 0;
	motorAPID->target = motorAPID->count;
	motorBPID->target = motorBPID->count;
	mtr_stop();
	// Also clear backup
	backup->mtrADir = DIR_FWD;
	backup->mtrBDir = DIR_FWD;
	backup->mtrAPWM = 0;
	backup->mtrBPWM = 0;
	backup->servoDir = STRAIGHT;
}

void turn(float turning_angle) {
	if (abs(turning_angle) < MAX_ORI_ERR)		// Too small, turn may not be accurate
		return;
	if ((turning_angle < 0) || (turning_angle >= 360))	// Invalid turning angle
		return;

	float target_ori = (*ori) + turning_angle;
	while (target_ori >= 360) target_ori -= 360;
	while (target_ori < 0) target_ori += 360;

	uint8_t near_0 = 0;
	if ((target_ori < MAX_ORI_ERR) || (target_ori > 360 - MAX_ORI_ERR)) {
		near_0 = 1;
	}

	uint8_t mtr_dir = 1;	// 0: stop, 1: fwd, 2: bck

	while (mtr_dir) {
		if (turning_angle <= 180) {	// Turn left
			if (mtr_dir == 1) {
				turnServo(LEFT);
				mtr_SetParamAndMove(motorA, DIR_FWD, 1500);
				mtr_SetParamAndMove(motorB, DIR_FWD, 1500);
				mtr_dir = 2;
			}
			else {
				turnServo(RIGHT);
				mtr_SetParamAndMove(motorA, DIR_BCK, 1500);
				mtr_SetParamAndMove(motorB, DIR_BCK, 1500);
				mtr_dir = 1;
			}
		}
		else {						// Turn right
			if (mtr_dir == 1) {
				turnServo(RIGHT);
				mtr_SetParamAndMove(motorA, DIR_FWD, 1500);
				mtr_SetParamAndMove(motorB, DIR_FWD, 1500);
				mtr_dir = 2;
			}
			else {
				turnServo(LEFT);
				mtr_SetParamAndMove(motorA, DIR_BCK, 1500);
				mtr_SetParamAndMove(motorB, DIR_BCK, 1500);
				mtr_dir = 1;
			}
		}

		// Poll orientation value and break if needed
		if (!near_0) {
			for (int i = 0; i < 250; i++) {
				if (abs((*ori) - target_ori) < MAX_ORI_ERR) {
					mtr_dir = 0;
					break;
				}
				osDelay(2);
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
			for (int i = 0; i < 250; i++) {
				if (((*ori) > bound_lo) || ((*ori) < bound_hi)) {
					mtr_dir = 0;
					break;
				}
				osDelay(2);
			}
		}
		mtr_stop();
	}
	mtr_stop();
}
/*
 * mtr_dir = { 0: stop, 1: fwd, 2: bck }
 * turning_angle only accept 90 (left) or 270 (right)
 */
void carTurn(uint8_t mtr_dir, float turning_angle) {
	// Check validity of parameters
	if ((mtr_dir < 1) || (mtr_dir > 2))
		return;
	if ((turning_angle != 90) && (turning_angle != 270))
		return;

	// Calculate target orientation
	float target_ori = (*ori) + turning_angle;
	while (target_ori >= 360) target_ori -= 360;
	while (target_ori < 0) target_ori += 360;

	// Adjustments for near 0 degree target orientation
	uint8_t near_0 = 0;
	float bound_lo, bound_hi;
	if ((target_ori < MAX_ORI_ERR) || (target_ori > 360 - MAX_ORI_ERR)) {
		near_0 = 1;
		if (target_ori > 350) {
			bound_lo = target_ori - MAX_ORI_ERR;
			bound_hi = target_ori + MAX_ORI_ERR - 360;
		}
		else {
			bound_lo = target_ori - MAX_ORI_ERR + 360;
			bound_hi = target_ori + MAX_ORI_ERR;
		}
	}

	// Pre-turning adjustments
	if ((turning_angle == 90) && (mtr_dir == 1)) {
		mtr_mov_cm(5.0, 5.0);
	}
	else if ((turning_angle == 270) && (mtr_dir == 1)) {
		mtr_mov_cm(9.0, 9.0);
	}
	else if ((turning_angle == 270) && (mtr_dir == 2)) {
		mtr_mov_cm(11.5, 11.5);
	}
	else if ((turning_angle == 90) && (mtr_dir == 2)) {
		mtr_mov_cm(10.5, 10.5);
	}
	mtr_stop();
	osDelay(200);
	// Start servo and motor in turn direction
	if (((turning_angle == 90) && (mtr_dir == 1)) || ((turning_angle == 270) && (mtr_dir == 2))) {
		turnServo(LEFT);
	}
	else {
		turnServo(RIGHT);
	}
	osDelay(200);
	if (mtr_dir == 1) {
		mtr_SetParamAndMove(motorA, DIR_FWD, 1800);
		mtr_SetParamAndMove(motorB, DIR_FWD, 1800);
	}
	else {
		mtr_SetParamAndMove(motorA, DIR_BCK, 2000);
		mtr_SetParamAndMove(motorB, DIR_BCK, 2000);
	}

	// Polling orientation and break when target reached
	while (1) {
		if (!near_0) {
			if (abs((*ori) - target_ori) < MAX_ORI_ERR) {
				break;
			}
		}
		else {
			if (((*ori) > bound_lo) || ((*ori) < bound_hi)) {
				break;
			}
		}
		osDelay(2);
	}
	mtr_stop();

	// Post-turning adjustments
	if ((turning_angle == 90) && (mtr_dir == 1)) {
		mtr_mov_cm(-11.5, -11.5);
	}
	else if ((turning_angle == 270) && (mtr_dir == 1)) {
		mtr_mov_cm(-9.3, -9.3);
	}
	else if ((turning_angle == 270) && (mtr_dir == 2)) {
		mtr_mov_cm(-6.0, -6.0);
	}
	else if ((turning_angle == 90) && (mtr_dir == 2)) {
		mtr_mov_cm(-8.2, -8.2);
	}
}

/*
 * @brief Run instruction inst
 * @retval Distance moved in cm (linear). If execute turn, always return 0 even if SOSBack is called.
 */
float executeInstruction(Instruction* inst, CompleteError* cpltErr) {
	float retval;
	if (inst->type == INST_TYPE_GOSTRAIGHT) {
		mtr_mov_cm((float)inst->val, (float)inst->val);
		retval = ((float)(motorAPID->count + motorBPID->count) / 2) / CNT_PER_CM;
	}
	else if (inst->type == INST_TYPE_TURN) {
		float turning_angle = (float)inst->val;
		if ((turning_angle >= 0) && (turning_angle < 360)) {
			turn(turning_angle);
		}
		else if (turning_angle < 0) {
			carTurn(2, turning_angle + 360);
		}
		else {
			carTurn(1, turning_angle - 360);
		}
		retval = 0;
	}
	else {
		return 0;
	}
	//cpltErr->finished = 1;
	if (cpltErr->type == CPLTERR_TYPE_UNDEFINED) {
		cpltErr->type = CPLTERR_TYPE_CPLT;
	}
	return retval;
}
