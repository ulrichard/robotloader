/* ****************************************************************************
 *                           _______________________
 *                          \| ROBOT ARM  SYSTEM |/
 *                            \_-_-_-_-_-_-_-_-_-_/         
 * ----------------------------------------------------------------------------
 * ------------------- [c]2006 / 2007 - AREXX ENGINEERING ---------------------
 * -------------------------- http://www.arexx.com/ ---------------------------
 * ****************************************************************************
 * File: RobotArmUart.c
 * Version: 1.0
 * Target: Robotarm - ATMEGA64 @16.384MHz
 * Author(s): Dominik S. Herwald
 * ****************************************************************************
 * Description:
 *
 * The Robotarm UART Library. (UART = "Universal Aynchronous Receiver Transceiver")
 *
 * Hint: You should better leave all this as it is if you just started with
 * C programming, but it is a good idea to read the comments and review the
 * code, it will help you to understand C programming for AVR better.
 *
 * Of course you are free to add new functions and improvements to this
 * library and make them available to the public on the Internet.
 * Please use the changelog at the end of this file to document your
 * changes. And add your name (or nickname) to any new function or 
 * modification you added! E.g. a "modified by <nickname> at <date>" is 
 * always a good idea to show other users where and what you changed the 
 * source code!
 *
 * ****************************************************************************
 * CHANGELOG AND LICENSING INFORMATION CAN BE FOUND AT THE END OF THIS FILE!
 * ****************************************************************************
 */
 
/*****************************************************************************/
// Includes:

#include "RobotArmUart.h"



/*****************************************************************************/
// UART transmit functions:

/**
 * Write a single character to the UART.
 *
 * Example:
 *
 *			writeChar('C');
 *			writeChar('P');
 *			writeChar('R');
 *			writeChar('\n');
 *			// '\n' is a special code for the "new line" character!
 *			writeChar('0');
 *			writeChar(48); // 48 is ASCII code for '0'
 *			writeChar(49); // 1
 *			writeChar(50); // 2
 *			writeChar(51); // 3
 *			//...
 *
 *			would output:
 *			CPR
 *			00123
 *
 */
void writeChar(char ch)
{
    while (!(UCSR1A & (1<<UDRE1)));
    UDR1 = (uint8_t)ch;
}

/**
 * Writes a null terminated string or buffer from SRAM to UART.
 * Make sure that it really IS null terminated!
 * ("null terminated" means that the string has a null (=0) at the end.
 * this is automatically added to it by the compiler when you put the
 * string in double quotes like this: writeString("test");  )
 *
 * ATTENTION: This fills up SRAM Memory with the
 * strings, even if they are constant and never changed.
 * If you want to write constant text strings to the UART, better use
 * writeNStringP(const uint8_t *pstring) (s. below), which reads the
 * text from flash program memory and does not fill up the SRAM with
 * the string data!
 *
 * Example:
 *
 *			writeString("Robotarm System\n");
 *
 */
void writeString(char *string)
{
	while(*string)
		writeChar(*string++);
}
		
/**
 * Writes a null terminated string from flash program memory to UART.
 * You can use the macro writeString_P(STRING); , this macro
 * ensures that the String is stored in program memory only!
 * Otherwise you need to use PSTR("your string") from AVRLibC for this. 
 *
 * Example:
 *
 *			writeNStringP(PSTR("Robotarm System\n"));
 *
 *			// There is also a Macro that makes life easier and
 *			// you can simply write:
 *			writeString_P("Robot ARm System\n");
 *
 */
void writeNStringP(const char *pstring)
{

    uint8_t c;
    for (;(c = pgm_read_byte_near(pstring++));writeChar(c));
	
}


/**
 * Writes a string with specified length and offset from SRAM to UART.
 * If it is a null terminated string, output will be stopped at the
 * end! It does not need to be null terminated, but it is recommended
 * to use only null terminated strings/buffers, otherwise the function could
 * output any SRAM memory data stored after the string until it reaches a 0
 * or the specified length.
 *
 * Example:
 *							  
 *			writeStringLength("RP6 Robot Sytem\n",16,0);
 *			// would output: "RP6 Robot Sytem\n"
 *			writeStringLength("RP6 Robot Sytem\n",11,4);
 *			// would output: "Robot System"
 * 			writeStringLength("RP6 Robot Sytem\n",40,4);
 *			// would output: "Robot System\n"
 *			// No matter if the specified length is 40 characters!
 *
 */
void writeStringLength(char *string, uint8_t length, uint8_t offset)
{
	for(string = &string[offset]; *string && length; length--)
		writeChar(*string++);
}

/**
 * Write a number (with specified base) to the UART.
 *
 * Example:
 *
 *			// Write a hexadecimal number to the UART:
 *			writeInteger(0xAACC,16);
 *			// Instead of 16 you can also write "HEX" as this is defined in the
 *			// RobotArmBaseLib.h :
 *			writeInteger(0xAACC, HEX);
 *			// Other Formats:
 *			writeInteger(1024,DEC);  	// Decimal
 *			writeInteger(044,OCT);		// Ocal
 *			writeInteger(0b11010111,BIN); // Binary
 */
void writeInteger(int16_t number, uint8_t base)
{
	char buffer[17];
	itoa(number, &buffer[0], base);
	writeString(&buffer[0]);
}

/**
 * Same as writeInteger, but with defined length.
 * This means this routine will add leading zeros to the number if length is
 * larger than the actual value or cut the upper digits if length is smaller
 * than the actual value.
 *
 * Example:
 *
 *			// Write a hexadecimal number to the UART:
 *			writeIntegerLength(0xAACC, 16, 8);
 *			// Instead of 16 you can also write "HEX" as this is defined in the
 *			// RobotArmBaseLib.h :
 *			writeIntegerLength(0xAACC, HEX, 8);
 *			// Other Formats:
 *			writeIntegerLength(1024,DEC,6);  	// Decimal
 *			writeIntegerLength(044,OCT,4);		// Ocal
 *			writeIntegerLength(0b11010111,BIN,8); // Binary
 */
void writeIntegerLength(int16_t number, uint8_t base, uint8_t length)
{
	char buffer[17];
	itoa(number, &buffer[0], base);
	int8_t cnt = length - strlen(buffer);
	if(cnt > 0) {
		for(; cnt > 0; cnt--, writeChar('0'));
		writeString(&buffer[0]);
	}
	else 
		writeStringLength(&buffer[0],length,-cnt);
}

/*****************************************************************************/
// UART receive functions:

volatile char uart_receive_buffer[UART_RECEIVE_BUFFER_SIZE];
uint8_t buffer_pos;
uint8_t uart_receive_bytes;
volatile uint8_t uart_status;


ISR(USART1_RX_vect)
{
	volatile char recChar = UDR1;
	if(uart_status == UART_BUISY) {
		uart_receive_buffer[buffer_pos++] = recChar;
		if(buffer_pos >= uart_receive_bytes)
			uart_status = UART_DATA_AVAILABLE;
	}
}

/**
 *
 */
void receiveBytes(uint8_t numberOfBytes)
{
	buffer_pos = 0;
	uart_receive_bytes = numberOfBytes;
	uart_status = UART_BUISY;
}

/**
 *
 */
void waitUntilReceptionComplete(void)
{
	while(getUARTReceiveStatus() == UART_BUISY);
}

/**
 *
 */
void copyReceivedBytesToBuffer(char *buffer)
{
	uint8_t i = 0;
	while(i < uart_receive_bytes) {
		buffer[i] = uart_receive_buffer[i];
		i++;
	}
	uart_status = UART_READY;
}

/**
 *
 */
void stopReception(void)
{
	uart_receive_bytes = 0;
	uart_status = UART_READY;
}

/**
 *
 */
void receiveBytesToBuffer(uint8_t numberOfBytes, char *buffer)
{
	receiveBytes(numberOfBytes);
	waitUntilReceptionComplete();
	copyReceivedBytesToBuffer(&buffer[0]);
}



/******************************************************************************
 * Additional info
 * ****************************************************************************
 * Changelog:
 * - v. 1.0 (initial release) 10.04.2007 by Dominik S. Herwald
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
