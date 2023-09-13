/*
 * imu.c
 *
 *  Created on: Aug 31, 2023
 *      Author: Duriana
 */
#include "imu.h"
#include "oled.h"
#include "stdlib.h"

/* Global variables */
/*----------------------------------------------------------------*/

I2C_HandleTypeDef* hi2c1_ptr;	// Pointer to hi2c1 object in main
uint8_t cur_bank = 4;					// Current user bank

/*----------------------------------------------------------------*/

/* Global config values (may change) */
/*----------------------------------------------------------------*/

/* Accelerometer */
/*+++++++++++++++++++++++++++++++++++*/

/* Enable Accel DLPF (default 1) */
uint8_t ACCEL_FCHOICE_val = 1;

/* Digital Low pass filter config (default 0) */
uint8_t ACCEL_DLPFCFG_val = 0;

/* Accel sample rate = 1125/(1+ACCEL_SMPLRT_DIV)Hz
 * Only applicable when ACCEL_FCHOICE = 1
 * Default [11:0] = 0
 */
uint16_t ACCEL_SMPLRT_DIV_val = 0;

/*+++++++++++++++++++++++++++++++++++*/

/* Gyroscope */
/*+++++++++++++++++++++++++++++++++++*/

/* Gyro scale */
uint8_t GYRO_FS_SEL_val = 0;

/* Enable Gyro DLPF (default 1) */
uint8_t GYRO_FCHOICE_val = 1;

/* Digital Low pass filter config (default 0) */
uint8_t GYRO_DLPFCFG_val = 1;

/* Gyro sample rate = 1.1 kHz/(1+GYRO_SMPLRT_DIV[7:0])
 * Only applicable when GYRO_FCHOICE = 1
 * Default: 0x00
*/
uint8_t GYRO_SMPLRT_DIV_val = 0;

/*+++++++++++++++++++++++++++++++++++*/

/*----------------------------------------------------------------*/

uint8_t imu_init(I2C_HandleTypeDef* hi2c_ptr) {
	hi2c1_ptr = hi2c_ptr;

	// Reset chip
	if (!write_one_byte(0, B0_PWR_MGMT_1, 0x80 | 0x41)) return 1;

	cur_bank = get_cur_bank();

	osDelay(100);
	uint8_t who_am_i = read_one_byte(0, 0);
	if (who_am_i != 0xEA) { 	// read WHO_AM_I register, should receive 0xEA
		return 2;
	}

	// Wake up chip
	if (!write_one_byte(0, B0_PWR_MGMT_1, ~0x40 & 0x41)) return 3;

	// Enables ODR start-time alignment when any of the following registers is written: GYRO_SMPLRT_DIV,
	// ACCEL_SMPLRT_DIV_1, ACCEL_SMPLRT_DIV_2, I2C_MST_ODR_CONFIG.
	if (!write_one_byte(2, B2_ODR_ALIGN_EN, 0x01)) return 4;

	// Set accel low pass filter
	if (!write_one_byte(2, B2_ACCEL_CONFIG, (ACCEL_DLPFCFG_val << 3) | ACCEL_FCHOICE_val)) return 5;

	// Set gyro low pass filter and scale
	if (!write_one_byte(2, B2_GYRO_CONFIG_1, (((GYRO_DLPFCFG_val << 2) | GYRO_FS_SEL_val) << 1) | GYRO_FCHOICE_val)) return 6;

	// Set accel sample rate divider
	if (ACCEL_SMPLRT_DIV_val > 0x0FFF) ACCEL_SMPLRT_DIV_val = 0x0FFF;
	if (!write_one_byte(2, B2_ACCEL_SMPLRT_DIV_1, (uint8_t)(ACCEL_SMPLRT_DIV_val >> 8))) return 7;
	if (!write_one_byte(2, B2_ACCEL_SMPLRT_DIV_2, (uint8_t)(ACCEL_SMPLRT_DIV_val & 0x00FF))) return 8;

	// Set gyro sample rate divider
	if (!write_one_byte(2, B2_GYRO_SMPLRT_DIV, GYRO_SMPLRT_DIV_val)) return 9;

	// Caliberate accelerometer (eliminate bias)
	accel_caliberate();

	// Caliberate gyroscope (eliminate bias)
	gyro_caliberate();

	return 0;
}

uint8_t read_one_byte(uint8_t bank, uint8_t regAddr) {

	if (!select_bank(bank)) return 0;
	uint8_t buf[1];

	uint8_t reg = regAddr;

	if (HAL_I2C_Master_Transmit(hi2c1_ptr, IMU_I2C_ADDR, &reg, 1, HAL_MAX_DELAY) == HAL_OK) {
		if (HAL_I2C_Master_Receive(hi2c1_ptr, IMU_I2C_ADDR, buf, 1, HAL_MAX_DELAY) == HAL_OK) {
			return buf[0];
		}
	}
	return 0;
}

uint8_t write_one_byte(uint8_t bank, uint8_t regAddr, uint8_t data) {

	if (!select_bank(bank)) return 0;
	uint8_t buf[2];
	buf[0] = regAddr;
	buf[1] = data;

	if (HAL_I2C_Master_Transmit(hi2c1_ptr, IMU_I2C_ADDR, buf, 2, HAL_MAX_DELAY) == HAL_OK) {
		return 1;
	}
	return 0;
}

uint8_t select_bank(uint8_t bank) {

	if (bank > 3) return 0;
	if (cur_bank == bank) return 1;
	uint8_t buf[2];
	buf[0] = REG_BANK_SEL;
	buf[1] = bank << 4;

	if (HAL_I2C_Master_Transmit(hi2c1_ptr, IMU_I2C_ADDR, buf, 2, HAL_MAX_DELAY) == HAL_OK) {
		cur_bank = bank;
		return 1;
	}
	return 0;
}

uint8_t get_cur_bank() {
	uint8_t buf[1];

	uint8_t reg = REG_BANK_SEL;

	if (HAL_I2C_Master_Transmit(hi2c1_ptr, IMU_I2C_ADDR, &reg, 1, HAL_MAX_DELAY) == HAL_OK) {
		if (HAL_I2C_Master_Receive(hi2c1_ptr, IMU_I2C_ADDR, buf, 1, HAL_MAX_DELAY) == HAL_OK) {
			return (buf[0] >> 4) & 0x3;
		}
	}
	return 4;
}

int16_t read_raw_accel_x() {
	int16_t raw = (int16_t)((read_one_byte(0, B0_ACCEL_XOUT_H) << 8) | read_one_byte(0, B0_ACCEL_XOUT_L));
	return raw;
}

float read_accel_x() {
	int16_t raw = read_raw_accel_x();
	float result = ((float)raw / ACCEL_SENSITIVITY_SCALE) * G;
	return result;
}

int16_t read_raw_accel_y() {
	int16_t raw = (int16_t)((read_one_byte(0, B0_ACCEL_YOUT_H) << 8) | read_one_byte(0, B0_ACCEL_YOUT_L));
	return raw;
}

float read_accel_y() {
	int16_t raw = read_raw_accel_y();
	float result = ((float)raw / ACCEL_SENSITIVITY_SCALE) * G;
	return result;
}

int16_t read_raw_accel_z() {
	int16_t raw = (int16_t)((read_one_byte(0, B0_ACCEL_ZOUT_H) << 8) | read_one_byte(0, B0_ACCEL_ZOUT_L));
	return raw;
}

float read_accel_z() {
	int16_t raw  = read_raw_accel_z();
	float result = ((float)raw / ACCEL_SENSITIVITY_SCALE) * G;
	return result;
}

void read_accel(AccelGyroResult* accel) {
	accel->x = read_accel_x();
	accel->y = read_accel_y();
	accel->z = read_accel_z();
}

/*
 * Assumption: Device is initialize on flat surface (ax = ay = 0, az = g)
 */
void accel_caliberate() {
	int32_t accel_bias_x = 0;
	int32_t accel_bias_y = 0;
	int32_t accel_bias_z = 0;
	for (int i = 0; i < 100; i++) {
		accel_bias_x += read_raw_accel_x();
		accel_bias_y += read_raw_accel_y();
		accel_bias_z += read_raw_accel_z();
	}
	accel_bias_x /= 100;
	accel_bias_y /= 100;
	accel_bias_z /= 100;
	accel_bias_z -= ACCEL_SENSITIVITY_SCALE;
	// Measured scale: 16384 lsb/g. Offset scale: 0.98 mg/lsb. Convert by offset = - measurement / 16
	accel_bias_x = -accel_bias_x / 16;
	accel_bias_y = -accel_bias_y / 16;
	accel_bias_z = -accel_bias_z / 16;
	// Get the preloaded offset and add with our custom bias
	int16_t accel_offset_x = (int16_t)(read_one_byte(1, B1_XA_OFFS_H) << 8 | read_one_byte(1, B1_XA_OFFS_L)) >> 1;
	int16_t accel_offset_y = (int16_t)(read_one_byte(1, B1_YA_OFFS_H) << 8 | read_one_byte(1, B1_YA_OFFS_L)) >> 1;
	int16_t accel_offset_z = (int16_t)(read_one_byte(1, B1_ZA_OFFS_H) << 8 | read_one_byte(1, B1_ZA_OFFS_L)) >> 1;

	accel_bias_x += accel_offset_x;
	accel_bias_y += accel_offset_y;
	accel_bias_z += accel_offset_z;

	// Separate accel_bias into 2 part: [14:7] and [6:0]
	uint8_t accel_bias_x_h = (uint8_t)(accel_bias_x >> 7);
	uint8_t accel_bias_x_l = (uint8_t)(accel_bias_x << 1);
	uint8_t accel_bias_y_h = (uint8_t)(accel_bias_y >> 7);
	uint8_t accel_bias_y_l = (uint8_t)(accel_bias_y << 1);
	uint8_t accel_bias_z_h = (uint8_t)(accel_bias_z >> 7);
	uint8_t accel_bias_z_l = (uint8_t)(accel_bias_z << 1);
	// Write to registers
	write_one_byte(1, B1_XA_OFFS_H, accel_bias_x_h);
	write_one_byte(1, B1_XA_OFFS_L, accel_bias_x_l);
	write_one_byte(1, B1_YA_OFFS_H, accel_bias_y_h);
	write_one_byte(1, B1_YA_OFFS_L, accel_bias_y_l);
	write_one_byte(1, B1_ZA_OFFS_H, accel_bias_z_h);
	write_one_byte(1, B1_ZA_OFFS_L, accel_bias_z_l);
}

int16_t read_raw_gyro_x() {
	int16_t raw = (int16_t)((read_one_byte(0, B0_GYRO_XOUT_H) << 8) | read_one_byte(0, B0_GYRO_XOUT_L));
	return raw;
}

float read_gyro_x() {
	int16_t raw = read_raw_gyro_x();
	float result = (float)raw / GYRO_SENSITIVITY_SCALE;
	return result;
}

int16_t read_raw_gyro_y() {
	int16_t raw = (int16_t)((read_one_byte(0, B0_GYRO_YOUT_H) << 8) | read_one_byte(0, B0_GYRO_YOUT_L));
	return raw;
}

float read_gyro_y() {
	int16_t raw = read_raw_gyro_y();
	float result = (float)raw / GYRO_SENSITIVITY_SCALE;
	return result;
}

int16_t read_raw_gyro_z() {
	int16_t raw = (int16_t)((read_one_byte(0, B0_GYRO_ZOUT_H) << 8) | read_one_byte(0, B0_GYRO_ZOUT_L));
	return raw;
}

float read_gyro_z() {
	int16_t raw = read_raw_gyro_z();
	float result = (float)raw / GYRO_SENSITIVITY_SCALE;
	return result;
}

void read_gyro(AccelGyroResult* gyro) {
	gyro->x = read_gyro_x();
	gyro->y = read_gyro_y();
	gyro->z = read_gyro_z();
}

void gyro_caliberate() {
	int32_t gyro_bias_x = 0;
	int32_t gyro_bias_y = 0;
	int32_t gyro_bias_z = 0;
	for (int i = 0; i < 100; i++) {
		gyro_bias_x += read_raw_gyro_x();
		gyro_bias_y += read_raw_gyro_y();
		gyro_bias_z += read_raw_gyro_z();
	}
	gyro_bias_x /= 100;
	gyro_bias_y /= 100;
	gyro_bias_z /= 100;
	// Step size for offset register: 0.0305 dps/LSB; Offset = Measurement / 4;
	gyro_bias_x = -gyro_bias_x / 4;
	gyro_bias_y = -gyro_bias_y / 4;
	gyro_bias_z = -gyro_bias_z / 4;
	/*gyro_bias_x = -gyro_bias_x * 2;
	gyro_bias_y = -gyro_bias_y * 2;
	gyro_bias_z = -gyro_bias_z * 2;*/
	// Separate gyro_bias into 2 parts: [15:8] and [7:0]
	uint8_t gyro_bias_x_h = (uint8_t)(gyro_bias_x >> 8);
	uint8_t gyro_bias_x_l = (uint8_t)gyro_bias_x;
	uint8_t gyro_bias_y_h = (uint8_t)(gyro_bias_y >> 8);
	uint8_t gyro_bias_y_l = (uint8_t)gyro_bias_y;
	uint8_t gyro_bias_z_h = (uint8_t)(gyro_bias_z >> 8);
	uint8_t gyro_bias_z_l = (uint8_t)gyro_bias_z;
	// Write to registers
	write_one_byte(2, B2_XG_OFFS_USRH, gyro_bias_x_h);
	write_one_byte(2, B2_XG_OFFS_USRL, gyro_bias_x_l);
	write_one_byte(2, B2_YG_OFFS_USRH, gyro_bias_y_h);
	write_one_byte(2, B2_YG_OFFS_USRL, gyro_bias_y_l);
	write_one_byte(2, B2_ZG_OFFS_USRH, gyro_bias_z_h);
	write_one_byte(2, B2_ZG_OFFS_USRL, gyro_bias_z_l);
}

float calcOri(uint32_t* lastSampleTime, float lastSampleOri) {
	float angular_speed = 0;
	angular_speed += read_gyro_z();
	if (abs(angular_speed) < MIN_ANG_SPD)
		angular_speed = 0;
	uint32_t curTime = HAL_GetTick();
	uint16_t dt = curTime - (*lastSampleTime);
	*lastSampleTime = curTime;
	float result = lastSampleOri + angular_speed * dt / 1000;
	while (result >= 360)
		result -= 360;
	while (result < 0)
		result += 360;
	return result;
}
