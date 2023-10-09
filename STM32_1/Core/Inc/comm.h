/*
 * comm.h
 *
 *	Communication and Instruction parsing module
 *
 *
 *	Communication protocol:
 *	++++++++++++++++++++++++++++++++++++++++++++
 *
 *	UART, 8-bit, no parity, 115200 bps
 *
 *
 *	Instruction parsing
 *	++++++++++++++++++++++++++++++++++++++++++++
 *
 *	Receive: 32 bits (4 bytes)
 *	----------------------------
 *
 *	RX[31:30] 	: 	Type[1:0]
 *	RX[29:24] 	: 	Reserved
 *	RX[23:8] 	: 	Val[15:0]
 *	RX[7:0]		:	InstructionID[7:0]
 *
 *	Type:
 *	10	:	Go straight <Val> cm
 *	11	:	Turn <Val> degree
 *
 *	Val (signed int):
 *	For going straight, Val = distance to move (in cm); positive = forward, negative = backward
 *	For turning,
 *		Val = degree to turn. Range: [0, 360) (inclusive of 0 but not 360) (Turn on the spot, slow and will introduce drift)
 *		Val = 360 + 90		--> Car turn (forward, to left, face left)
 *		Val = 360 + 270		--> Car turn (forward, to right, face right)
 *		Val = -90			--> Car turn (backward, to left, face right)
 *		Val = -270			--> Car turn (backward, to right, face left)
 *
 *	Command decode and validity check will be done in executeInstruction()
 *
 *	InstructionID (unsigned int):
 *	ID of instruction receive OR Corresponding ID of Complete / Error
 *	(1 - 255, starting from 1 and increase by 1 for each instruction)
 *
 *	Transmit:
 *	----------------------------
 *
 *	1/ ACK: 32 bits (4 bytes) [Any transmission with MSB == 0 is ACK]
 *
 *	TX[31:24]	:	0x41	(Ascii 'A')
 *	TX[23:16]	:	0x43	(Ascii 'C')
 *	TX[15:8]	:	0x4B	(Ascii 'K')
 *	TX[7:0]		:	InstructionID[7:0] (See line 32)
 *
 *
 *	2/ Complete / Error: 32 bits (4 bytes)
 *	TX[31:30]	:	10 for complete, 11 for error
 *	TX[29:19]	:	POS_X[10:0]
 *	TX[18:8]	:	POS_Y[10:0]
 *	TX[7:0]		:	InstructionID[7:0] (See line 32)
 *
 *	POS_X (signed int):
 *	x-coordinate of car position (in cm) (signed int)
 *
 *	POS_Y (signed int):
 *	y-coordinate of car position (in cm) (signed int)
 *
 *
 *	Communication order for 1 instruction
 *	++++++++++++++++++++++++++++++++++++++++++++
 *
 *	RPI sends instruction -> STM32 replies with ACK (1) -> STM32 sends Complete / Error ->
 *	RPI replies with ACK (1) -> Repeat from 1st step (instructions to be sent one at a time)
 *	(1): After waiting for <UART_ACK_MAX_DELAY> ms, if no ACK is received, transmitter resends data.
 *
 *
 *  Created on: Sep 16, 2023
 *      Author: Duriana
 */

#ifndef INC_COMM_H_
#define INC_COMM_H_

#include "stm32f4xx_hal.h"

#define INST_TYPE_GOSTRAIGHT 0
#define INST_TYPE_TURN 1
#define INST_TYPE_UNDEFINED 2
#define CPLTERR_TYPE_CPLT 0
#define CPLTERR_TYPE_ERR 1
#define CPLTERR_TYPE_UNDEFINED 2

#define UART_ACK_MAX_DELAY 5000
#define UART_PACKET_SIZE 5

/* Struct for instructions (Received) */
typedef struct {
	uint8_t id;
	uint8_t type;
	int16_t val;
} Instruction;

/*
 * Struct for Complete / Error (Transmit)
 * New instance init only by newCpltErr
*/
typedef struct {
	uint8_t id;
	uint8_t type;
	int16_t pos_x;
	int16_t pos_y;
	uint8_t finished;		// Only assigned to by executeInstruction()
} CompleteError;

void comm_init(UART_HandleTypeDef* uart, Instruction* curInstObjRef, CompleteError* cpltErrObjRef);
HAL_StatusTypeDef uart_send();
HAL_StatusTypeDef uart_receive(const uint8_t* uartbuf);
uint8_t getCurInstId();
uint8_t newCpltErr(uint8_t id);

#endif /* INC_COMM_H_ */
