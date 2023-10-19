/*
 * motors.h
 *
 *  Created on: Aug 30, 2023
 *      Author: Duriana
 */

#ifndef INC_MOTORS_H_
#define INC_MOTORS_H_

#include "stm32f4xx_hal.h"
#include "cmsis_os.h"
#include "comm.h"

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

/* Suspend status */
#define SUS_OFF 0
#define SUS_STOP 1
#define SUS_BACK 2
#define SUS_STOPPID 3

/* Timer used: htim8, channels as defined below */
#define PWMA_TIM_CH TIM_CHANNEL_1
#define PWMB_TIM_CH TIM_CHANNEL_2

/* Limits */
#define MAX_SPEED 7199		// = TIM8.ARR
#define MIN_SPEED 1200		// Min PWM val for the car to move
#define MAX_PID_ERR 50
#define MAX_ORI_ERR 1.5f

/* Other constants */
#define PULSE_PER_REV 330 	// Pulse Per revolution generated in each channel of the encoder
#define CNT_PER_CM 73.266f
#define SOSBACK_DIST_CNT -800	// Count change when run mtr_SOSBack()

/* Basic GPIO Set / Clear for GPIO_out Pins */

#define MOTOR_AIN1_Set() HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_SET)	// AIN1 = 1
#define MOTOR_AIN1_Clr() HAL_GPIO_WritePin(MOTOR_AIN1_Port, MOTOR_AIN1_Pin, GPIO_PIN_RESET)	// AIN1 = 0

#define MOTOR_AIN2_Set() HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_SET)	// AIN2 = 1
#define MOTOR_AIN2_Clr() HAL_GPIO_WritePin(MOTOR_AIN2_Port, MOTOR_AIN2_Pin, GPIO_PIN_RESET) // AIN2 = 0

#define MOTOR_BIN1_Set() HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_SET)	// BIN1 = 1
#define MOTOR_BIN1_Clr() HAL_GPIO_WritePin(MOTOR_BIN1_Port, MOTOR_BIN1_Pin, GPIO_PIN_RESET)	// BIN1 = 0

#define MOTOR_BIN2_Set() HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_SET)	// BIN2 = 1
#define MOTOR_BIN2_Clr() HAL_GPIO_WritePin(MOTOR_BIN2_Port, MOTOR_BIN2_Pin, GPIO_PIN_RESET) // BIN2 = 0

/* Struct for storing Motor's data */

typedef struct {
	uint8_t suspend;
	uint8_t dir;			// Direction of motor
	uint32_t pwmVal;		// pwm value to control motor speed
} MotorData;

/* Struct for storing data used in motor PID control */

typedef struct {
	int16_t count;       		// Counter (signed value)
	int16_t target;		 		// target angle of rotation,
	int16_t error;           	// error between target and actual
	int32_t error_area;  		// area under error - to calculate I for PI implementation
	int32_t error_old;	 		// to calculate D for PID control
	int32_t millisOld;			// to calculate I and D for PID control
	float Kp;
	float Kd;
	float Ki;
} MotorPIDData;

/* Struct for storing current / backup motor and servo motor */

typedef struct {
	uint8_t servoDir;
	uint8_t mtrADir;
	uint8_t mtrBDir;
	uint32_t mtrAPWM;
	uint32_t mtrBPWM;
} MotorServoStatus;

/* Motor control functions */
void mtr_init(TIM_HandleTypeDef* pwm_htimPtr, TIM_HandleTypeDef* encodeA_htimPtr, TIM_HandleTypeDef* encodeB_htimPtr,
		MotorData* mtrA, MotorData* mtrB, MotorPIDData* mtrAPID, MotorPIDData* mtrBPID, MotorServoStatus* backupObj,
		float* orientation, osSemaphoreId_t* oriSemHandlePtr);
void mtrA_init(int16_t target, int16_t Kp, float Kd, float Ki, uint8_t reset_timer);
void mtrB_init(int16_t target, int16_t Kp, float Kd, float Ki, uint8_t reset_timer);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void mtrA_mov(uint8_t direction, uint16_t speed);
void mtrB_mov(uint8_t direction, uint16_t speed);
void mtr_stop();
void mtr_mov(MotorData* motor);
void mtr_SetParamAndMove(MotorData* motor, uint8_t param_dir, uint32_t param_pwmVal);
void mtr_suspend(uint8_t mode);
void mtr_continue();
float mtr_SOSBack();
void mtr_mov_cnt(int target_A, int target_B);
double mtr_mov_cm(double target);
void PID_Control(MotorData* motor, MotorPIDData* motorPID);
void stopPID();
void turn(float target_ori);
void carTurn(uint8_t mtr_dir, float turning_angle);
float executeInstruction(Instruction* inst, CompleteError* cpltErr);

#endif /* INC_MOTORS_H_ */
