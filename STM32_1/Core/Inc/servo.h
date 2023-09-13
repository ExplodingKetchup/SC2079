/*
 * servo.h
 *
 *  Created on: Sep 7, 2023
 *      Author: Duriana
 */

#ifndef CORE_INC_SERVO_H_
#define CORE_INC_SERVO_H_

#include "stm32f4xx_hal.h"

#define LEFT 0
#define STRAIGHT 1
#define RIGHT 2

void turnLeft();
void turnRight();
void turnStraight();
void turnServo(uint8_t);
void servoInit(TIM_HandleTypeDef* htim);
#endif /* CORE_INC_SERVO_H_ */
