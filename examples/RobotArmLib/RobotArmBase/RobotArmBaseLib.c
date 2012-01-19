/* ****************************************************************************
 *                           ____________________________
 *                           \| ROBOT ARM SYSTEM |/
 *                            \_-_-_-_-_-_-_-_-_-_/         
 * ----------------------------------------------------------------------------
 * ------------------- [c]2010 - AREXX ENGINEERING ---------------------
 * -------------------------- http://www.arexx.com/ ---------------------------
 * ****************************************************************************
 * File: RobotArmBaseLib.c
 * Version: 1.0
 * Target: Robotarm - ATMEGA64 @16.384MHz
 * Author(s): Huy Nguyen 
 			  Hein Wielink
 * ****************************************************************************
 * Description:
 *
 * This is the Robotarm Library - it contains the following functions:
 * - Processor initialisation
 * - LED Control
 * - Beeper control
 * - A/D Convertor (Battery Voltage sensors, current measurement )
 * - Servo Control 
 * - Timing functions 
 *
 * PLEASE ALSO READ THE ROBOT ARM MANUAL! THERE YOU WILL FIND EXAMPLE
 * CODE AND SOME FURTHER EXPLANATIONS!
 *
 * In other parts of this library RobotArmUart.c, 
 * you can find UART communication.
 *
 * -----
 * Hint: You should better leave all this as it is if you just started with
 * C programming, but it is a very good idea to read the comments and review 
 * the code, it will help you to understand C programming for AVR better!
 * -----
 *
 * For the experienced users: 
 * This code works OK, but it is not optimal! There is a lot potential for 
 * tuning! 
 * Well, this leaves some tasks for you and this is what makes most 
 * fun: To improve the excisting!  
 *
 * Of course you are free to add new functions and improvements to this
 * library and make them available to the public on the Internet e.g. on
 * our Forum!
 * Please use the changelog at the end of this file to document your
 * changes! And add your name to any new function or modification you added! 
 * E.g. a "modified by <name> at <date>" is always a good idea to show 
 * other users where and WHAT you changed in the source code!
 *
 * It is a good idea to make your own includeable libraries instead of
 * changing this library - of course only if this is possible.
 *
 * Or create your own complete library with all specific functions you need.
 * This code is GPL'd - s. license at the end of this file!
 *
 * ****************************************************************************
 * CHANGELOG AND LICENSING INFORMATION CAN BE FOUND AT THE END OF THIS FILE!
 * ****************************************************************************
 */

/*****************************************************************************/
// Includes:

#include "RobotArmBaseLib.h"

int BeepSound;
volatile uint16_t delay_timer;
volatile uint8_t ms_timer;
volatile stopwatches_t stopwatches;
volatile uint8_t feeler_timer;

/*****************************************************************************/
//Timer Interrupts:

//Timer2 (100uS Timer) 
ISR(TIMER2_COMP_vect){
	
		delay_timer++;

		if(ms_timer++ >= 10) { // 10 * 100µs = 1ms
		// 16bit Stopwatches:
		if(stopwatches.watches & STOPWATCH1)
			stopwatches.watch1++;
		if(stopwatches.watches & STOPWATCH2)
			stopwatches.watch2++;
		if(stopwatches.watches & STOPWATCH3)
			stopwatches.watch3++;
		if(stopwatches.watches & STOPWATCH4)
			stopwatches.watch4++;
		if(stopwatches.watches & STOPWATCH5)
			stopwatches.watch5++;
		if(stopwatches.watches & STOPWATCH6)
			stopwatches.watch6++;
		if(stopwatches.watches & STOPWATCH7)
			stopwatches.watch7++;
		if(stopwatches.watches & STOPWATCH8)
			stopwatches.watch8++;

		ms_timer=0;
		}
	
	}


/*****************************************************************************/
// LEDs:



/*****************************************************************************
// Set Power LED on red
 * Example:
 *
 *			PowerLEDred();
 *			// Power led = RED
 */

void PowerLEDred(void){
PORTG |= LED_RED;
PORTG &= ~LED_GREEN;	
}

/*****************************************************************************/
// Set Power LED on green
void PowerLEDgreen(void){	
PORTG &= ~LED_RED;
PORTG |= LED_GREEN;		
}

/*****************************************************************************/
// Set Power LED on orange
void PowerLEDorange(void){	
PORTG |= LED_RED;
PORTG |= LED_GREEN;	
}

/*****************************************************************************/
// Set Power LED off
void PowerLEDoff(void){
PORTG &= ~LED_RED;
PORTG &= ~LED_GREEN;	
}

/*****************************************************************************/
/* Delay with the help of timer2.
 * msleep(1) delays for *about* 1ms! Not exaclty, as we do not use assembly routines
 * anywhere in this library!
 *
 * This is a blocking routine, which means that the processor
 * will loop in this routine and (except for interrupts) the
 * normal program flow is stopped!
 * Thus you should use the Stopwatch functions wherever you can!
 *
 * Example:
 *		msleep(1); // delay 1 * 1ms = 1ms 
 *		msleep(10); // delay 10 * 1ms = 10ms 
 *		msleep(100); // delay 100 * 1ms = 100ms 
 *		// The maximum delay is:
 *		msleep(65535); // delay 65535 * 1ms = 65535ms = 1 min. 5s   
 */  

void sleep(uint8_t time)
{
	for (delay_timer = 0; delay_timer < time;);
}


void mSleep(uint16_t time)
{
	
	while (time--) sleep(10);
}


/*****************************************************************************/
// Set Beep sound on
void setBeepsound(void)
{
	//Clear OC0 on compare match, set OC0 at BOTTOM,
	TCCR0 =   (1 << WGM00) | (1 << WGM01) 
			| (0 << COM00) | (1 << COM01)   
			| (0 << CS02)  | (1 << CS01) | (0 << CS00);
}

/*****************************************************************************/
// Set Beep sound off
void clearBeepsound(void)
{	
	//Normal port operation, OC0 disconnected.
	TCCR0 =   (1 << WGM00) | (1 << WGM01) 
			| (0 << COM00) | (0 << COM01)   
			| (0 << CS02)  | (1 << CS01) | (0 << CS00);
}


/*****************************************************************************/
// Change Beep Sound
/* Example:

 *		While(true)
 *		{
 *			setBeepsound();
 *			changeBeepsound(150);
 *			mSleep(1000);
 *			changeBeepsound(200);
 *			mSleep(1000);
 *			changeBeepsound(255);
 *			mSleep(1000);
 *		}
 */		
	
		

void changeBeepsound(int PWMvalue)
{
	OCR0=PWMvalue;
}


/*****************************************************************************/
/************************** Extern Keyboard **********************************/
/*****************************************************************************/

int scan_keyboard(void){

	int i,mask,x;

	PORTA = 0x10;
	mSleep(2);	
	mask = 1;
	x = PINA;

	for(i=0;i<4;i++){
		if((x & mask) == mask)
            return i + 1;
        mask <<= 1;
	
	}
	
	PORTA = 0x20;
	mSleep(2);	
	mask = 1;
	x = PINA;

	for(i=0;i<4;i++){
		if((x & mask) == mask)
            return i + 5;
        mask <<= 1;
	}
	
	PORTA = 0x40;
	mSleep(2);	
	mask = 1;
	x = PINA;

	for(i=0;i<4;i++){
		if((x & mask) == mask)
            return i + 9;
        mask <<= 1;
	}
	
	PORTA = 0x80;
	mSleep(2);	
	mask = 1;
	x = PINA;

	for(i=0;i<4;i++){
		if((x & mask) == mask)
            return i + 13;
        mask <<= 1;
	}
	
	return 0;
}


/*****************************************************************************/
/************************** Servo Control   **********************************/
/*****************************************************************************/


//Measuring Current 


int readADC (int channel)
{
	int low = 0 ,high = 0, result = 0;	
	ADCL=0x00;	// Reset ADCL
	ADCH=0x00;	// Reset ADCH
	ADMUX&=0xE0;	// Reset Channel  
	ADMUX|=channel;	// Set Channel 
	
	mSleep(10);			//waits to become stable after changing the channel
	
	ADCSRA |= 0x40; // start Analog to Digital Conversion 
	while((ADCSRA & 0x10)==0);	// Wait for the AD conversion to complete
	ADCSRA |= 0x10; //set the bit to clear ADIF flag 
	
	low = ADCL;
  	high = ADCH ;
	result = ((high<<8)|low ); 
	return result;
}

// Declare storage areas for EEPROM
//	VALUE EEMEM ee_DEFAULT;
uint16_t EEMEM ee_Start_Position[7];


// Write default values from servos, to the EEPROM 
void write_Values_EE (void){
	// To write EEPROM from SRAM
	eeprom_write_block((const void *)&Start_Position,(void *)&ee_Start_Position, sizeof(Start_Position));
}

// Read default values from servos, from the EEPROM 
void Read_Values_EE (void){

	// To read EEPROM back to SRAM
	eeprom_read_block((void *)&Start_Position, (const void*)&ee_Start_Position, sizeof(Start_Position));
	
	if(Start_Position[1]  == -1)  Start_Position[1] = 1500;  
	if(Start_Position[2]  == -1)  Start_Position[2] = 1500;  
	if(Start_Position[3]  == -1)  Start_Position[3] = 1500;  
	if(Start_Position[4]  == -1)  Start_Position[4] = 1500;  
	if(Start_Position[5]  == -1)  Start_Position[5] = 1500;  
	if(Start_Position[6]  == -1)  Start_Position[6] = 1500;  

	Start_position(); 	

}


/*****************************************************************************/
// Power off servo's 
void Power_Servos (void)
{
PORTG &=~SERVO_POWER;
}

/*****************************************************************************/
// Power off servo's 

void Power_Off_Servos (void)
{
PORTG |=SERVO_POWER;
}



/*****************************************************************************/
// Set servo motors in normal position
void Start_position(void){

	Pos_Servo_1=Start_Position [1];
	Pos_Servo_2=Start_Position [2];
	Pos_Servo_3=Start_Position [3];
	Pos_Servo_4=Start_Position [4];
	Pos_Servo_5=Start_Position [5];
	Pos_Servo_6=Start_Position [6];
	
}

/*****************************************************************************/
// Move Servo's 

void Move (uint8_t Servo, uint16_t Value) 
{

	switch (Servo) 
	{
	case 1:	Pos_Servo_1 = Start_Position[1] + Value; break; 
	case 2:	Pos_Servo_2 = Start_Position[2] + Value; break; 
	case 3:	Pos_Servo_3 = Start_Position[3] + Value; break; 
	case 4:	Pos_Servo_4 = Start_Position[4] + Value; break; 
	case 5:	Pos_Servo_5 = Start_Position[5] + Value; break; 
	case 6:	Pos_Servo_6 = Start_Position[6] + Value; break; 
	}
}


/*****************************************************************************/
/* Move 
* Example:
*
* 	s_Move(6, 500,2);
*		1 - Servo 6
*		2 - (Startpostion + 500) = 2ms (right)
*		3 - (speed = 2) 
*/


void s_Move (uint8_t Servo, int16_t D_Value, uint16_t Speed) 
{
	int16_t Actual_position=2700;  
	
	switch (Servo) 
	{
	case 1:	Actual_position = Pos_Servo_1 - Start_Position[1]; break; 
	case 2:	Actual_position = Pos_Servo_2 - Start_Position[2]; break; 
	case 3:	Actual_position = Pos_Servo_3 - Start_Position[3]; break; 
	case 4:	Actual_position = Pos_Servo_4 - Start_Position[4]; break; 
	case 5:	Actual_position = Pos_Servo_5 - Start_Position[5]; break; 
	case 6:	Actual_position = Pos_Servo_6 - Start_Position[6]; break; 
	}
	
	if (Actual_position > D_Value )
	{ 
		while (Actual_position > D_Value)
		{
			Actual_position--; 
			Move(Servo, Actual_position);
			mSleep(Speed);
		}
		return; 
	}
	
	
	if (Actual_position < D_Value )
	{ 
		while (Actual_position < D_Value)
		{
			Actual_position++; 
			Move(Servo, Actual_position);
			mSleep(Speed);
		}
		return;
	}	
	
	
	
}



/*****************************************************************************/
// Total init for Robotarm 
/*
 * Initialise the Robot Controller - ALWAYS CALL THIS FIRST!
 * The Processor will not work correctly otherwise.
 * (If you don't implement your own init routine!)
 *
 * Example:
 *
 *			int16_t main(void)
 *			{
 *				initRobotBase();
 *
 *				// ... your application code
 *
 *				while(true);
 *				return 0;
 *			}
 *
 */
void initRobotBase(void){
	
	
	/*****************************************************************************/
	portInit();		// Setup port directions and initial values.
					// THIS IS THE MOST IMPORTANT STEP!
	
	/*****************************************************************************/
	cli();			// Disable global interrupts
	
	
	/*****************************************************************************/
	// setADC
	// AVCC with external capacitor at AREF pin
	ADMUX = (1<<REFS1)|(1<<REFS0)|(0<<ADLAR)|(0<<MUX4)|(0<<MUX3)|(0<<MUX2)|(0<<MUX1)|(0<<MUX0);
	// ADC Enable, ADC Start Conversion, Auto Trigger Enable, Division Factor 8
	ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADATE)|(0<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); 
	// Free Running mode
	ADCSRB  = (0<<ADTS2)|(0<<ADTS1)|(0<<ADTS0);
	//ADCSRA |= (1<<ADIF);
	

	/*****************************************************************************/
	// Initialize Timer 0 -  PWM Beeper
	TCCR0 =   (1 << WGM00) | (1 << WGM01) 
			| (0 << COM00) | (0 << COM01)   
			| (0 << CS02)  | (1 << CS01) | (0 << CS00);
			
	//TIMSK = (0 << OCIE0)|(0<<TOIE0) ;
	OCR0  = 255;

	/*****************************************************************************/
	// Initialize Timer 2 -  100µs cycle for Delays and Stopwatches:
	TCCR2 =   (0 << WGM20) | (1 << WGM21) 	| (0 << COM20) | (0 << COM21) 
			   | (0 << CS22)  | (1 << CS21) | (0 << CS20);	   
	OCR2  = 204;
	

	/*****************************************************************************/
	// **** Timer/Counter 1 initialization (timer for servo's) ****
	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	// Compare C Match Interrupt: Off

	TCCR1A=0xA8;

	// Clock source: System Clock
	// Clock value: 20480 kHz
	//
	//    16.484.000 / 2*8*20480 = 50 Hz. 
	//	  (Datasheet page 130) 
	//
	//
	// Mode: Ph. & fr. cor. PWM top=ICR1
	TCCR1B=0x12;
	ICR1 = 20480;

	TCNT1H=0x00;
	TCNT1L=0x00;

	// OC1A output: Non-Inv.
	// OC1B output: Non-Inv.
	// OC1C output: Non-Inv.

	/*****************************************************************************/
	// *** Timer/Counter 3 initialization (timer for servo's)***

	// Compare A Match Interrupt: Off
	// Compare B Match Interrupt: Off
	// Compare C Match Interrupt: Off
	TCCR3A=0xA8;

	// Clock source: System Clock
	// Clock value: 20480 kHz
	//
	//    16.484.000 / 2*8*20480 = 50 Hz. 
	//	  (Datasheet page 130) 
	//
	//
	// Mode: Ph. & fr. cor. PWM top=ICR3
	
	TCCR3B=0x12;
	ICR3 = 20480;
	TCNT3H=0x00;
	TCNT3L=0x00;

	// OC3A output: Non-Inv.
	// OC3B output: Non-Inv.
	// OC3C output: Non-Inv.
	
	Read_Values_EE ();		//Read default values for the servos from the EEprom 
							//And store in SRAM	

	/*****************************************************************************/
	// UART:
	UBRR1H = UBRR_BAUD_LOW >> 8;	// Setup UART: Baudrate is Low Speed
	UBRR1L = (uint8_t) UBRR_BAUD_LOW;
	UCSR1A = 0x00;
    UCSR1C = (1<<UCSZ11)|(1<<UCSZ10);
    UCSR1B = (1 << TXEN1) | (1 << RXEN1) | (1 << RXCIE1);
	

	
	
	/*****************************************************************************/
	//Enable Compare output Timer 2 
	TIMSK |= (1 << OCIE2);

	sei();
	
	PowerLEDred();		//Power Led red
	mSleep(1000);		//wait for 1 sec. 
	PowerLEDgreen();	//Power Led green (OK) 
}


/******************************************************************************
 * Additional info
 * ****************************************************************************
 * Changelog:
 * - v. 1.0 (initial release) 27.05.2010 by Huy Nguyen 
 											Hein Wielink
 *
* ****************************************************************************
 * Bugs, feedback, questions and modifications can be posted on the AREXX Forum
 * on http://www.arexx.com/forum/ !
 * Of course you can also write us an e-mail to: info@arexx.nl
 * AREXX Engineering may publish updates from time to time on AREXX.com!
 * ****************************************************************************
 * - LICENSE -
 * GNU GPL v2 (http://www.gnu.org/licenses/gpl.txt, a local copy can be found
 * on the Robotarm CD!)
 * This program is free software. You can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as published
 * by the Free Software Foundation.
 * ****************************************************************************
 */

/*****************************************************************************/
// EOF



