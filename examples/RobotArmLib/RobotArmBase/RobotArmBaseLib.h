/* ****************************************************************************
 *                           _______________________
 *                           \| ROBOT ARM SYSTEM |/
 *                            \_-_-_-_-_-_-_-_-_-_/         
 * ----------------------------------------------------------------------------
 * ------------------- [c]2006 / 2007 - AREXX ENGINEERING ---------------------
 * -------------------------- http://www.arexx.com/ ---------------------------
 * ****************************************************************************
 * File: RobotArmBaseLib.h
 * Version: 1.0
 * Target: Robot ARM Base - ATMEGA64 @16.384MHz
 * Author(s): Hein Wielink
			  Huy Nguyen 
 * ****************************************************************************
 * Description:
 *
 * ****************************************************************************
 * CHANGELOG AND LICENSING INFORMATION CAN BE FOUND AT THE END OF THIS FILE!
 * ****************************************************************************
 */


/*****************************************************************************/
// Includes:
#pragma once

#include "RobotArmBase.h"	// General Robot ARM Base definitions
#include "RobotArmUart.h"		// Robotarm UART function lib
#include <avr/eeprom.h> 
#include <stdlib.h>


#ifdef __cplusplus
 extern "C" {
#endif 
/*****************************************************************************/
// Init:

void initRobotBase(void);

/*****************************************************************************/
// Status LEDs

void PowerLEDred(void);
void PowerLEDgreen(void);
void PowerLEDorange(void);
void PowerLEDoff(void);

/*****************************************************************************/
// Stopwatches:

#define STOPWATCH1 1
#define STOPWATCH2 2
#define STOPWATCH3 4
#define STOPWATCH4 8
#define STOPWATCH5 16
#define STOPWATCH6 32
#define STOPWATCH7 64
#define STOPWATCH8 128

/*
 * All are 16bit Stopwatches --> maximum value is 65.535 (16Bit) or ~65.535 seconds!
 */
typedef struct {
	volatile uint8_t watches;
	volatile uint16_t watch1;
	volatile uint16_t watch2;
	volatile uint16_t watch3;
	volatile uint16_t watch4;
	volatile uint16_t watch5;
	volatile uint16_t watch6;
	volatile uint16_t watch7;
	volatile uint16_t watch8;
} stopwatches_t;
extern volatile stopwatches_t stopwatches;

#define stopStopwatch1() stopwatches.watches &= ~STOPWATCH1
#define stopStopwatch2() stopwatches.watches &= ~STOPWATCH2
#define stopStopwatch3() stopwatches.watches &= ~STOPWATCH3
#define stopStopwatch4() stopwatches.watches &= ~STOPWATCH4
#define stopStopwatch5() stopwatches.watches &= ~STOPWATCH5
#define stopStopwatch6() stopwatches.watches &= ~STOPWATCH6
#define stopStopwatch7() stopwatches.watches &= ~STOPWATCH7
#define stopStopwatch8() stopwatches.watches &= ~STOPWATCH8

#define startStopwatch1() stopwatches.watches |= STOPWATCH1
#define startStopwatch2() stopwatches.watches |= STOPWATCH2
#define startStopwatch3() stopwatches.watches |= STOPWATCH3
#define startStopwatch4() stopwatches.watches |= STOPWATCH4
#define startStopwatch5() stopwatches.watches |= STOPWATCH5
#define startStopwatch6() stopwatches.watches |= STOPWATCH6
#define startStopwatch7() stopwatches.watches |= STOPWATCH7
#define startStopwatch8() stopwatches.watches |= STOPWATCH8

#define isStopwatch1Running() (stopwatches.watches & STOPWATCH1)
#define isStopwatch2Running() (stopwatches.watches & STOPWATCH2)
#define isStopwatch3Running() (stopwatches.watches & STOPWATCH3)
#define isStopwatch4Running() (stopwatches.watches & STOPWATCH4)
#define isStopwatch5Running() (stopwatches.watches & STOPWATCH5)
#define isStopwatch6Running() (stopwatches.watches & STOPWATCH6)
#define isStopwatch7Running() (stopwatches.watches & STOPWATCH7)
#define isStopwatch8Running() (stopwatches.watches & STOPWATCH8)

#define getStopwatch1() stopwatches.watch1
#define getStopwatch2() stopwatches.watch2
#define getStopwatch3() stopwatches.watch3
#define getStopwatch4() stopwatches.watch4
#define getStopwatch5() stopwatches.watch5
#define getStopwatch6() stopwatches.watch6
#define getStopwatch7() stopwatches.watch7
#define getStopwatch8() stopwatches.watch8

#define setStopwatch1(__VALUE__) stopwatches.watch1 = __VALUE__
#define setStopwatch2(__VALUE__) stopwatches.watch2 = __VALUE__
#define setStopwatch3(__VALUE__) stopwatches.watch3 = __VALUE__
#define setStopwatch4(__VALUE__) stopwatches.watch4 = __VALUE__
#define setStopwatch5(__VALUE__) stopwatches.watch5 = __VALUE__
#define setStopwatch6(__VALUE__) stopwatches.watch6 = __VALUE__
#define setStopwatch7(__VALUE__) stopwatches.watch7 = __VALUE__
#define setStopwatch8(__VALUE__) stopwatches.watch8 = __VALUE__



/*****************************************************************************/
// Delays

void sleep(uint8_t time);
void mSleep(uint16_t time);

/*****************************************************************************/
// Sensor


uint16_t getUbat(void);

/*****************************************************************************/
// Beeper

void setBeepsound(void);
void clearBeepsound(void);
void changeBeepsound(int PWMvalue);

/*****************************************************************************/
// Extern keypad 

int scan_keyboard(void);

/*****************************************************************************/
// ADC
extern int Current_1; 
extern int Current_2; 
extern int Current_3; 
extern int Current_4; 
extern int Current_5; 
extern int Current_6; 

int readADC (int channel);


/*****************************************************************************/
// Servo

extern uint16_t  Start_Position[7];

void write_Values_EE (void);
void Read_Values_EE (void);
void Power_Servos(void);
void Power_Off_Servos(void);
void Start_position(void);
void Move (uint8_t Servo, uint16_t Value);
void s_Move (uint8_t Servo, int16_t D_Value, uint16_t Speed);

#ifdef __cplusplus
 } // extern "C" 
#endif 


/******************************************************************************
 * Additional info
 * ****************************************************************************
 * Changelog:
 * 
 *  ---> changes are documented in the file "RobotArmBaseLib.c"
 *
 * ****************************************************************************
 * Bugs, feedback, questions and modifications can be posted on the AREXX Forum
 * on http://www.arexx.com/forum/ !
 * Of course you can also write us an e-mail to: info@arexx.nl
 * AREXX Engineering may publish updates from time to time on AREXX.com!
 * ****************************************************************************
 * - LICENSE -
 * GNU GPL v2 (http://www.gnu.org/licenses/gpl.txt, a local copy can be found
 * on the RobotArm CD!)
 * This program is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 * ****************************************************************************
 */

/*****************************************************************************/
// EOF
