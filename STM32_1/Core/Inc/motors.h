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
#define MIN_SPEED 1200		// Min PWM val for the car to move
#define MAX_PID_ERR 50
#define MAX_ORI_ERR 3

/* Other constants */
#define PULSE_PER_REV 330 	// Pulse Per revolution generated in each channel of the encoder
#define CNT_PER_CM 73.266f

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
	uint8_t dir;		// Direction of motor
	uint32_t pwmVal;		// pwm value to control motor speed
} MotorData;

/* Struct for storing data used in motor PID control */
/*typedef struct {
	uint32_t count;       		// Counter (signed value)
	int16_t angle;      		// angle of rotation, in degree resolution
	int16_t target_angle; 		// target angle of rotation,
	int16_t error;           	// error between target and actual
	int32_t error_area;  		// area under error - to calculate I for PI implementation
	int32_t error_old;	 		// to calculate D for PID control
	int32_t millisOld;			// to calculate I and D for PID control
	int16_t Kp;
	float Kd;
	float Ki;
} MotorPIDData; */
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

/* Motor control functions */
void mtr_init(TIM_HandleTypeDef* pwm_htimPtr, TIM_HandleTypeDef* encodeA_htimPtr, TIM_HandleTypeDef* encodeB_htimPtr,
		MotorData* mtrA, MotorData* mtrB, MotorPIDData* mtrAPID, MotorPIDData* mtrBPID, osSemaphoreId_t* oriSemHandlePtr);
void mtrA_init(int16_t target, int16_t Kp, float Kd, float Ki, uint8_t reset_timer);
void mtrB_init(int16_t target, int16_t Kp, float Kd, float Ki, uint8_t reset_timer);
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim);
void mtrA_mov(uint8_t direction, uint16_t speed);
void mtrB_mov(uint8_t direction, uint16_t speed);
void mtr_mov(MotorData* motor);
void mtr_stop();
void mtr_mov_cnt(int target_A, int target_B);
void mtr_mov_cm(float target_A, float target_B);
void PID_Control(MotorData* motor, MotorPIDData* motorPID);
void turn(float target_ori, float* orientation);

#endif /* INC_MOTORS_H_ */
