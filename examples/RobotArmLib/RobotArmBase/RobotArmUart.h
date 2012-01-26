/* ****************************************************************************
 *                           ______________________
 *                           \| ROBOT ARM SYSTEM |/
 *                            \_-_-_-_-_-_-_-_-_-_/         >>> BASE CONTROLLER
 * ----------------------------------------------------------------------------
 * ------------------- [c]2006 / 2007 - AREXX ENGINEERING ---------------------
 * -------------------------- http://www.arexx.com/ ---------------------------
 * ****************************************************************************
 * File: RobotArmUart.h
 * Version: 1.0
 * Target: Robotarm - ATMEGA64 @16.384MHz
 * Author(s): Dominik S. Herwald
 * ****************************************************************************
 * Description:
 * The Robotarm uart function Library header file. Detailled description
 * of each function can be found in the RobotArmUart.c file!
 *
 * ****************************************************************************
 * CHANGELOG AND LICENSING INFORMATION CAN BE FOUND AT THE END OF THIS FILE!
 * ****************************************************************************
 */

#ifndef ROBOTARMUART_H
#define ROBOTARMUART_H

/*****************************************************************************/
// Includes:

#include <avr/pgmspace.h> 	// Program memory (=Flash ROM) access routines.
#include <stdlib.h>			// C standard functions (e.g. itoa...)
#include <avr/io.h>			// I/O Port definitions
#include <avr/interrupt.h>	// Interrupt macros (e.g. cli(), sei())

#include <string.h>

#ifdef __cplusplus
 extern "C" {
#endif
/*****************************************************************************/
// UART

void writeChar(char ch);
void writeStringLength(char *data, uint8_t length, uint8_t offset);
void writeString(char *data);
void writeNStringP(const char *pstring);
#define writeString_P(__pstr) writeNStringP((PSTR(__pstr)))

#define HEX 16
#define DEC 10
#define OCT 8
#define BIN 2
void writeInteger(int16_t number, uint8_t base);
void writeIntegerLength(int16_t number, uint8_t base, uint8_t length);

// RX:
extern volatile uint8_t uart_status;

#define UART_RECEIVE_BUFFER_SIZE 32 // Default buffer size is 32!
#define UART_BUFFER_OK 0
#define UART_BUFFER_OVERFLOW 1

char readChar(void);
uint8_t readChars(char *buf, uint8_t numberOfChars);
uint8_t getBufferLength(void);
void clearReceptionBuffer(void);

#ifdef __cplusplus
 } // extern "C"
#endif

#endif

/******************************************************************************
 * Additional info
 * ****************************************************************************
 * Changelog:
 *
 *  ---> changes are documented in the file "RobotArmuart.c"
 *
 * ****************************************************************************
 * Bugs, feedback, questions and modifications can be posted on the AREXX Forum
 * on http://www.arexx.com/forum/ !
 * Of course you can also write us an e-mail to: info@arexx.nl
 * AREXX Engineering may publish updates from time to time on AREXX.com!
 * ****************************************************************************
 * - LICENSE -
 * GNU GPL v2 (http://www.gnu.org/licenses/gpl.txt)
 * This program is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 * ****************************************************************************
 */

/*****************************************************************************/
// EOF
