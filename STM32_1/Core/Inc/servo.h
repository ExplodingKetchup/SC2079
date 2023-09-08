/*
 * servo.h
 *
 *  Created on: Sep 7, 2023
 *      Author: Duriana
 */

#ifndef CORE_INC_SERVO_H_
#define CORE_INC_SERVO_H_

#include "stm32f4xx_hal.h"

void turnLeft();
void turnRight();
void turnStraight();
void turn(int);
void servoInit(TIM_HandleTypeDef* htim);
#endif /* CORE_INC_SERVO_H_ */
