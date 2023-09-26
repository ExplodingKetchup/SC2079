/*
 * comm.c
 *
 *  Created on: Sep 16, 2023
 *      Author: Duriana
 */

#include "comm.h"
#include "cmsis_os.h"

/* Global variables */

uint8_t instructionId = 0;
uint8_t ack_tx = 0;				// ID of Complete / Error ACKed by RPI

uint8_t uartbuf[4];

UART_HandleTypeDef* huart3Ptr;
Instruction* curInstPtr;
CompleteError* cpltErrPtr;

/* Private function prototypes */

void uart_ack(uint8_t id);

/* Functions */

void comm_init(UART_HandleTypeDef* uart, Instruction* curInstObjRef, CompleteError* cpltErrObjRef) {
	instructionId = 0;
	ack_tx = 0;

	huart3Ptr = uart;
	curInstPtr = curInstObjRef;
	cpltErrPtr = cpltErrObjRef;

	curInstPtr->id = 0;
	curInstPtr->type = INST_TYPE_UNDEFINED;
	curInstPtr->val = 0;

	cpltErrPtr->id = 0;
	cpltErrPtr->type = CPLTERR_TYPE_UNDEFINED;
	cpltErrPtr->pos_x = 0;
	cpltErrPtr->pos_y = 0;
	cpltErrPtr->finished = 1;

	HAL_UART_Receive_IT(huart3Ptr, (uint8_t*) uartbuf, 4);

	return;
}

HAL_StatusTypeDef uart_send() {
	if (cpltErrPtr->id == 0) {
		return HAL_ERROR;
	}
	if (ack_tx != cpltErrPtr->id - 1) {	// RPI is not expecting CompleteError with this id
		return HAL_ERROR;
	}
	uartbuf[0] = (((0x01 << 1) | (cpltErrPtr->type & 0x01)) << 6) | (uint8_t)((cpltErrPtr->pos_x >> 5) & 0x003F);
	uartbuf[1] = (uint8_t)((cpltErrPtr->pos_x & 0x1F) << 3) | (uint8_t)((cpltErrPtr->pos_y >> 8) & 0x0007);
	uartbuf[2] = (uint8_t)(cpltErrPtr->pos_y & 0xFF);
	uartbuf[3] = cpltErrPtr->id;
	while (ack_tx < cpltErrPtr->id) {
		while (HAL_UART_Transmit(huart3Ptr, (uint8_t*)uartbuf, 4, UART_ACK_MAX_DELAY) != HAL_OK) {
			osDelay(500);
		}
		// Waiting for ACK
		for (int i = 0; i < 10; i++) {
			if (ack_tx < cpltErrPtr->id) {
				osDelay(UART_ACK_MAX_DELAY / 10);
			}
			else if (ack_tx == cpltErrPtr->id) {	// RPI received cpltErr
				break;
			}
			else {
				return HAL_ERROR;
			}
		}
	}
	return HAL_OK;
}

void uart_ack(uint8_t id) {
	uartbuf[0] = 0x41;
	uartbuf[1] = 0x43;
	uartbuf[2] = 0x4B;
	uartbuf[3] = id;
	while (HAL_UART_Transmit(huart3Ptr, (uint8_t*)uartbuf, 4, UART_ACK_MAX_DELAY) != HAL_OK) {
		osDelay(500);
	}
	return;
}

HAL_StatusTypeDef uart_receive() {
	uint8_t id = uartbuf[3];
	if (uartbuf[0] == 0x41) {	// ACK from RPI
		ack_tx = id;
		return HAL_OK;
	}
	if (id == instructionId + 1) {		// Received instruction is correct in order
		curInstPtr->id = id;
		curInstPtr->type = (uartbuf[0] >> 6) & 0x01;
		curInstPtr->val = ((int16_t)uartbuf[2] << 8) | uartbuf[1];
		instructionId++;
		uart_ack(instructionId);
		return HAL_OK;
	}
	uart_ack(instructionId);
	return HAL_ERROR;
}

uint8_t getCurInstId() {
	return instructionId;
}

uint8_t newCpltErr(uint8_t id) {
	instructionId = 1;		// For testing only, delete
	if ((id == instructionId) && (ack_tx == id - 1)) {
		cpltErrPtr->id = id;
		cpltErrPtr->type = CPLTERR_TYPE_UNDEFINED;
		cpltErrPtr->pos_x = 0;
		cpltErrPtr->pos_y = 0;
		cpltErrPtr->finished = 0;
		return 1;
	}
	return 0;
}
