/* ****************************************************************************
 *                           ___________________________
 *                           \| Robotarm ROBOT SYSTEM |/
 *                            \_-_-_-_-_-_-_-_-_-_/         >>> BASE CONTROLLER
 * ----------------------------------------------------------------------------
 * ------------------- [c]2010 - AREXX ENGINEERING ---------------------
 * -------------------------- http://www.arexx.com/ ---------------------------
 * ****************************************************************************
 * File: RobotArmBase.h
 * Version: 1.00
 * Target: Robotarm Base - ATMEGA64 @16.384 MHz
 * Author(s): Hein Wielink
              Huy Nguyen 
 * ****************************************************************************
 * Description:
 * The RobotArm Base header file with general definitions. If you don't want
 * to include the complete RobotArmBaseLib because it is too large - then at least
 * include this file! It is already included in the RobotArmBaseLib, but can be
 * used without it!
 *
 * This file contains helpful definitions that simplify reading the sourcecode.
 * Most important are the default settings for Port and Direction registers!
 * Hint: You should better leave all this as it is, but it is a good idea to
 * read the comments, it will help you to understand C programming for AVR
 * better!
 */

#ifndef ROBOTARMBASE_H
#define ROBOTARMBASE_H


/*****************************************************************************/
// Useful definition for common routines that are used on several
// different devices (can switch register settings to match the target!):

#define DEVICE_ROBOTARMBASE

/*****************************************************************************/
// CPU Clock:

#ifdef F_CPU
#undef F_CPU
#endif
#ifndef F_CPU
#define F_CPU 16384000 //Base: 16.384MHz  - DO NOT CHANGE!
#endif

/*****************************************************************************/
// Includes:


#include <avr/io.h>			// I/O Port definitions
#include <avr/interrupt.h>	// Interrupt macros (e.g. cli(), sei())

/*****************************************************************************/
// Servo's 



#define Pos_Servo_1		OCR1A
#define Pos_Servo_2		OCR1B
#define Pos_Servo_3 	OCR1C
#define Pos_Servo_4 	OCR3A
#define Pos_Servo_5 	OCR3B
#define Pos_Servo_6 	OCR3C

#define Servo1			1
#define Servo2			2
#define Servo3 			3
#define Servo4 			4
#define Servo5 			5
#define Servo6 			6

#define max_current_servo1	60
#define max_current_servo2	60
#define max_current_servo3	60
#define max_current_servo4	80
#define max_current_servo5	150
#define max_current_servo6	150


/*****************************************************************************/
// I/O PORT pin definitions
// These definitions simplify reading and understanding the source code.
//
// ATTENTION: Initial value of port and direction registers should not
// be changed, if you do not exactly know what you are doing!
//
// Hints for DDRx and PORTx Registers:
// DDRxy = 0 and PORTxy = 0 ==> Input without internal Pullup
// DDRxy = 0 and PORTxy = 1 ==> Input with internal Pullup
// DDRxy = 1 and PORTxy = 0 ==> Output low
// DDRxy = 1 and PORTxy = 1 ==> Output high
// "=1" indicates that the appropriate bit is set.
//
// Example:
// #define INIT_DDRA 0b00010000
// #define INIT_PRTA 0b00000000
//
// This means that ALL ports on PortA are inputs without internal pullups
// except for PortA4, which is an output (E_INT1 signal in this case) and
// initial value is low.
//
// Binary value explanation:
// 0b00010000     = 16 in decimal system
//   ^      ^
// MSB      LSB      (MSB = Most Significant Bit, LSB = Least Significant Bit)
//
// The program should always call the macro "portInit();" FIRST! You can find
// it a bit below. Correct port initialisation is the most important step
// after a hardware reset!

// ---------------------------------------------------
// PORTA

#define EXT_IN_1			(1 << PINA0) //ADC0 (input)
#define EXT_IN_2			(1 << PINA1) //ADC1 (input)
#define EXT_IN_3			(1 << PINA2) //ADC2 (input) 
#define EXT_IN_4			(1 << PINA3) //ADC3 (input)
#define EXT_OUT_1			(1 << PINA4) //ADC4 (output)
#define EXT_OUT_2			(1 << PINA5) //ADC5 (output)
#define EXT_OUT_3			(1 << PINA6) //ADC6 (output)
#define EXT_OUT_4			(1 << PINA7) //ADC7 (output)

// Initial value of port and direction registers.
#define INIT_DDRA 0b11110000
#define INIT_PRTA 0b00000000


// ---------------------------------------------------
// PORTB

#define SS			(1 << PINB0) //Output
#define SCK 	   	(1 << PINB1) //Output
#define MOSI 	   	(1 << PINB2) //Output
#define MISO		(1 << PINB3) //Output
#define BEEPER		(1 << PINB4) //Output
#define P_SERVO1 		(1 << PINB5) // Output
#define P_SERVO2 		(1 << PINB6) // Input
#define P_SERVO3 		(1 << PINB7) // Output

// Initial value of port and direction registers.
#define INIT_DDRB 0b11111111
#define INIT_PRTB 0b00000000

// ---------------------------------------------------
// PORTC


#define A			(1 << PINC0) //Input
#define B			(1 << PINC1) //Input
#define C			(1 << PINC2) //Output
#define ADC_EN		(1 << PINC3) //Output
#define EYE_EN		(1 << PINC4) //Output
#define SW_2		(1 << PINC5) //Output
#define SW_3		(1 << PINC6) //Output
//#define DEFAULT		(1 << PINC7) //Output


// Initial value of port and direction registers.
#define INIT_DDRC 0b11111100
#define INIT_PRTC 0b00000000

// ---------------------------------------------------
// PORTD

#define SCL_PC		PIND0			//I2C CLOCK
#define SDA_PC		PIND1			//I2C DATA 
#define RXD1		(1 << PIND2)	//USART RX (Input)
#define TXD1		(1 << PIND3)	//USART TX (Output)
#define RESERVE_PD4	(1 << PIND4)	//EXTERN PIN PD4, (SPI)
#define RESERVE_PD5	(1 << PIND5)	//EXTERN PIN PD5, (SPI)
#define RESERVE_PD6	(1 << PIND6)	//EXTERN PIN PD6, (SPI)
#define RESERVE_PD7	(1 << PIND7)	//EXTERN PIN PD7, (SPI)

// Initial value of port and direction registers.
#define INIT_DDRD 0b00001011
#define INIT_PRTD 0b00000000


// ---------------------------------------------------
// PORTE

#define RXD0		(1 << PINE0)	//USART RX (Input)
#define TXD0		(1 << PINE1)	//USART TX (Output)
//#define DEFAULT	(1 << PINE2)	//default
#define P_SERVO4	(1 << PINE3)	//Servo 4 (Output)
#define P_SERVO5	(1 << PINE4)	//Servo 5 (Output)
#define P_SERVO6	(1 << PINE5)	//Servo 6 (Output) 
#define INT6_EXT	(1 << PINE6)	//Interrupt (input)
#define INT7_EXT	(1 << PINE7)	//Interrupt (input)

// Initial value of port and direction registers.
#define INIT_DDRE 0b11111010
#define INIT_PRTE 0b00000000


// ---------------------------------------------------
// PORTF

#define CUR_1	(1 << PINF0)	//Measure current servo 1 
#define CUR_2	(1 << PINF1)	//Measure current servo 2  
#define CUR_3	(1 << PINF2)	//Measure current servo 3 
#define CUR_4	(1 << PINF3)	//Measure current servo 4 
#define CUR_5	(1 << PINF4)	//Measure current servo 5  
#define CUR_6	(1 << PINF5)	//Measure current servo 6  
#define UBAT		(1 << PINF6)	//EXTERN PIN 
#define EXT_ADC		(1 << PINF7)	//EXTERN PIN 


// Initial value of port and direction registers.
#define INIT_DDRF 0b00000000
#define INIT_PRTF 0b00000000


// ---------------------------------------------------
// PORTA A/D Convertor channels

#define ADC_CURRENT_1			0
#define ADC_CURRENT_2			1
#define ADC_CURRENT_3			2
#define ADC_CURRENT_4			3
#define ADC_CURRENT_5			4
#define ADC_CURRENT_6			5
#define ADC_UBAT				6
#define ADC_EXT_ADC				7


// ---------------------------------------------------
// PORTG

#define LED_RED		(1 << PING0)	//Status Led Red 
#define LED_GREEN	(1 << PING1)	//Status Led Green 
#define RESERVE_PG2	(1 << PING2)	//Extern  PG2 
#define SERVO_POWER	(1 << PING3)	//Power Servo's 
//#define DEFAULT	(1 << PING4)	//DEFAULT

// Initial value of port and direction registers.
#define INIT_DDRG 0b00011111
#define INIT_PRTG 0b00001000




/*****************************************************************************/
// I/O Port init macro - always call this first! It is called first from
// initRobotBase() in the RobotArmBaseLib!
//
// Example:
// int main(void)
// {
// 		portInit();
// 		// ...
//		// your application
//		while(true);
//		return 0;
// }

#define portInit();	\
PORTA = INIT_PRTA;	\
PORTB = INIT_PRTB;	\
PORTC = INIT_PRTC;	\
PORTD = INIT_PRTD;	\
PORTE = INIT_PRTE;	\
PORTF = INIT_PRTF;	\
PORTG = INIT_PRTG;	\
DDRA = INIT_DDRA;	\
DDRB = INIT_DDRB;	\
DDRC = INIT_DDRC;	\
DDRD = INIT_DDRD;	\
DDRE = INIT_DDRE;	\
DDRF = INIT_DDRF;	\
DDRG = INIT_DDRG;

/*****************************************************************************/
// Some additional definitions/macros

// Boolean:
#define true 1
#define false 0
#define TRUE 1
#define FALSE 0

// Assembly and system macros:
#define nop() asm volatile("nop\n\t")
#define sysSleep() asm volatile("sleep\n\t")

/*****************************************************************************/
// Baudrates:

#define BAUD_LOW		38400  //Low speed - 38.4 kBaud
#define UBRR_BAUD_LOW	((F_CPU/(16*BAUD_LOW))-1)

#define BAUD_HIGH		500000 //High speed - 500 kBaud
#define UBRR_BAUD_HIGH	((F_CPU/(16*BAUD_HIGH))-1)

#endif
