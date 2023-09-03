/*
 * imu.h
 *
 *  Created on: Sep 1, 2023
 *      Author: Duriana
 */

#ifndef INC_IMU_H_
#define INC_IMU_H_

#include "stm32f4xx_hal.h"

/* I2C Slave Address */
#define IMU_I2C_ADDR 0xD0

/* Register addresses */
/* For more info please refer to https://invensense.tdk.com/wp-content/uploads/2021/10/DS-000189-ICM-20948-v1.5.pdf */

/* Measurement Units */
#define G 9.81

/* Selecting user banks [5:4] */
#define REG_BANK_SEL 0x7F

/* General config registers */
#define B0_PWR_MGMT_1 0x06
#define B2_ODR_ALIGN_EN 0x09

/* Accelerometer Registers */
#define B0_ACCEL_XOUT_H 0x2D
#define B0_ACCEL_XOUT_L 0x2E
#define B0_ACCEL_YOUT_H 0x2F
#define B0_ACCEL_YOUT_L 0x30
#define B0_ACCEL_ZOUT_H 0x31
#define B0_ACCEL_ZOUT_L 0x32
#define B2_ACCEL_CONFIG 0x14	// [2:1] = ACCEL_FS[1:0] = 00 (+- 2G)
#define B2_ACCEL_SMPLRT_DIV_1 0x10
#define B2_ACCEL_SMPLRT_DIV_2 0x11
#define B1_XA_OFFS_H 0x14
#define B1_XA_OFFS_L 0x15
#define B1_YA_OFFS_H 0x17
#define B1_YA_OFFS_L 0x18
#define B1_ZA_OFFS_H 0x1A
#define B1_ZA_OFFS_L 0x1B
#define ACCEL_SENSITIVITY_SCALE 16384
/* Gyroscope Registers */
#define B0_GYRO_XOUT_H 0x33
#define B0_GYRO_XOUT_L 0x34
#define B0_GYRO_YOUT_H 0x35
#define B0_GYRO_YOUT_L 0x36
#define B0_GYRO_ZOUT_H 0x37
#define B0_GYRO_ZOUT_L 0x38
#define B2_GYRO_CONFIG_1 0x01	// [2:1] = GYRO_FS_SEL[1:0] = 00
#define B2_GYRO_SMPLRT_DIV 0x00
#define B2_XG_OFFS_USRH 0x03
#define B2_XG_OFFS_USRL 0x04
#define B2_YG_OFFS_USRH 0x05
#define B2_YG_OFFS_USRL 0x06
#define B2_ZG_OFFS_USRH 0x07
#define B2_ZG_OFFS_USRL 0x08
#define GYRO_SENSITIVITY_SCALE 131

/* Struct to store accelerometer / gyroscope measurement results (in 3 axises) */
typedef struct {
	float x;
	float y;
	float z;
} AccelGyroResult;

/* Init */
uint8_t imu_init(I2C_HandleTypeDef* hi2c_ptr);

/* Basic read/write to registers */
uint8_t read_one_byte(uint8_t bank, uint8_t regAddr);
uint8_t write_one_byte(uint8_t bank, uint8_t regAddr, uint8_t data);
uint8_t select_bank(uint8_t bank);
uint8_t get_cur_bank();

/* Accelerometer */
int16_t read_raw_accel_x();
float read_accel_x();
int16_t read_raw_accel_y();
float read_accel_y();
int16_t read_raw_accel_z();
float read_accel_z();
void read_accel(AccelGyroResult* accel);
void accel_caliberate();

/* Gyroscope */
int16_t read_raw_gyro_x();
float read_gyro_x();
int16_t read_raw_gyro_y();
float read_gyro_y();
int16_t read_raw_gyro_z();
float read_gyro_z();
void read_gyro(AccelGyroResult* gyro);
void gyro_caliberate();


#endif /* INC_IMU_H_ */
