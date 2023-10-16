/*
 * servo.c
 *
 *  Created on: Sep 7, 2023
 *      Author: Duriana
 */
#include "servo.h"
#include "cmsis_os.h"

uint8_t cur_direction; // 0 as left, 1 as straight, 2 as right
TIM_HandleTypeDef* htim1Ptr;

void turnLeft(){
	htim1Ptr->Instance->CCR4=962;
	cur_direction = LEFT;
	osDelay(700);
}
void turnRight(){
	htim1Ptr->Instance->CCR4=2200;
	cur_direction = RIGHT;
	osDelay(700);
}
void turnStraight(){
	if(cur_direction == LEFT){
		htim1Ptr->Instance->CCR4=1548;
		osDelay(700);
	}else if(cur_direction == RIGHT){
		htim1Ptr->Instance->CCR4=1457;
		osDelay(700);
	}else{
		return;
	}
	cur_direction = STRAIGHT;
}
void turnServo(uint8_t target){
	if(target == LEFT){
		turnLeft();
	}else if(target == STRAIGHT){
		turnStraight();
	}else{
		turnRight();
	}
}
uint8_t getServoDir() {
	return cur_direction;
}
void servoInit(TIM_HandleTypeDef* htim){
	htim1Ptr = htim;
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
	cur_direction = 1;
	htim1Ptr->Instance->CCR4=2200;
	HAL_Delay(700);
	htim1Ptr->Instance->CCR4=1000;
	HAL_Delay(700);
	htim1Ptr->Instance->CCR4=1548;
	HAL_Delay(700);

}

