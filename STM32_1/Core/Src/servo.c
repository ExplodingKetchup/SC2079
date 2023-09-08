/*
 * servo.c
 *
 *  Created on: Sep 7, 2023
 *      Author: Duriana
 */
#include "servo.h"

int cur_direction, target_direction; // 0 as left, 1 as straight, 2 as right
TIM_HandleTypeDef* htim1Ptr;

void turnLeft(){
	if(cur_direction == 0){
		return;
	}else if(cur_direction == 1){
		htim1Ptr->Instance->CCR4=110;
		HAL_Delay(250);
	}else{
		// cur_direction == 2
		htim1Ptr->Instance->CCR4=145;
		HAL_Delay(250);
		htim1Ptr->Instance->CCR4=110;
		HAL_Delay(250);
	}
	cur_direction = 0;
}
void turnRight(){
	if(cur_direction == 0){
		htim1Ptr->Instance->CCR4=155;
		HAL_Delay(250);
		htim1Ptr->Instance->CCR4=200;
		HAL_Delay(250);
	}else if(cur_direction == 1){
		htim1Ptr->Instance->CCR4=200;
		HAL_Delay(250);
	}else{
		// cur_direction == 2
		return;
	}
	cur_direction = 2;
}
void turnStraight(){
	if(cur_direction == 0){
		htim1Ptr->Instance->CCR4=155;
		HAL_Delay(250);
	}else if(cur_direction == 2){
		htim1Ptr->Instance->CCR4=145;
		HAL_Delay(250);
	}else{
		return;
	}
	cur_direction = 1;
}
void turn(int target){
	target_direction = target;
	if(target == 0){
		turnLeft();
	}else if(target == 1){
		turnStraight();
	}else{
		turnRight();
	}
}
void servoInit(TIM_HandleTypeDef* htim){
	htim1Ptr = htim;
	HAL_TIM_PWM_Start(htim, TIM_CHANNEL_4);
	cur_direction = 1;
	target_direction = 1;
	htim1Ptr->Instance->CCR4=110;
	HAL_Delay(250);
	htim1Ptr->Instance->CCR4=155;
	HAL_Delay(250);
	htim1Ptr->Instance->CCR4=200;
	HAL_Delay(250);
	htim1Ptr->Instance->CCR4=145;
	HAL_Delay(250);
	htim1Ptr->Instance->CCR4=110;
	HAL_Delay(250);
	htim1Ptr->Instance->CCR4=155;
	HAL_Delay(250);

}

