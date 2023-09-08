/*
 * servo.c
 *
 *  Created on: Sep 7, 2023
 *      Author: Duriana
 */
int cur_direction, target_direction; // 0 as left, 1 as straight, 2 as right
void turnLeft(){
	if(cur_direction == 0){
		return;
	}else if(cur_direction == 1){
		htim1.Instance->CCR4=110;
		OS_Delay(250);
	}else{
		// cur_direction == 2
		htim1.Instance->CCR4=145;
		OS_Delay(250);
		htim1.Instance->CCR4=110;
		OS_Delay(250);
	}
	cur_direction = 0;
}
void turnRight(){
	if(cur_direction == 0){
		htim1.Instance->CCR4=155;
		OS_Delay(250);
		htim1.Instance->CCR4=200;
		OS_Delay(250);
	}else if(cur_direction == 1){
		htim1.Instance->CCR4=200;
		OS_Delay(250);
	}else{
		// cur_direction == 2
		return;
	}
	cur_direction = 2;
}
void turnStraight(){
	if(cur_direction == 0){
		htim1.Instance->CCR4=155;
		OS_Delay(250);
	}else if(cur_direction == 2){
		htim1.Instance->CCR4=145;
		OS_Delay(250);
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
void servoInit(){
	cur_direction = 1;
	target_direction = 1;
	htim1.Instance->CCR4=110;
	OS_Delay(250);
	htim1.Instance->CCR4=155;
	OS_Delay(250);
	htim1.Instance->CCR4=200;
	OS_Delay(250);
	htim1.Instance->CCR4=145;
	OS_Delay(250);
	htim1.Instance->CCR4=110;
	OS_Delay(250);
	htim1.Instance->CCR4=155;
	OS_Delay(250);

}

