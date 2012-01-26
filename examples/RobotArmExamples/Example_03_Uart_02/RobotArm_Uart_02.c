/* 
 * ****************************************************************************
 * Robotarm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm uart example 2
 * Author(s): Hein wielink
			  Huy Nguyen 
 * ****************************************************************************
 * Description:
 * This sample program shows how to use the Robotarm Library functions to receive
 * data with the serial interface. It is a "Question and Answer" program. 
 *
 * ############################################################################
 * The Robot does NOT move in this example! You can simply put it on a table
 * next to your PC and you should connect it to the PC via the USB Interface!
 * ############################################################################
 * ****************************************************************************
 */
 
/*****************************************************************************/
#include "RobotArmBaseLib.h"// The Robotarm Robot Library.
								// Always needs to be included!

int8_t question;

void outputTextStuff(void)
{
uint8_t x,y;
	for(x = 0; x < 53; x++)
	{
		writeStringLength("1234567890abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ",10,x);
		writeChar(' ');
		for(y = 0; y < x; y++)
			writeChar('#');
		mSleep(200); 
		writeChar('\n');
	}
}

void Leds_Beeper (void)

{

	changeBeepsound(150);
	setBeepsound();
	PowerLEDred();	
	mSleep(500); 
	
	changeBeepsound(200);
	PowerLEDgreen();	
	mSleep(500); 
	
	changeBeepsound(255);
	PowerLEDorange();
	mSleep(500); 
	
	PowerLEDoff();
	clearBeepsound();
}


void askAQuestion(void)
{
	uint8_t bytesToReceive = 1;
	
	clearReceptionBuffer();
	switch(question)
	{
		case 0:
			writeString_P("What's your name? (Enter 8 characters please)\n"); 
			bytesToReceive = 8; 
		break;
		case 1:
			writeString_P("How old are you? (Enter 2 characters please)\n"); 
			bytesToReceive = 2;
		break;
		case 2: 
			writeString_P("Do you want to see some Text output? (\"y\" or \"n\")\n"); 
		break;
		case 3: 
			writeString_P("Do you want to see blinking LEDs and hear the beeper sound? (\"y\" or \"n\")\n"); 
		break;
		case 4: 
			writeString_P("Do you want to do all this again? (\"y\" or \"n\")\n"); 
		break;
	}

	for(size_t i=0; i<100 && getBufferLength() < bytesToReceive; ++i)
		;	
	char receiveBuffer[bytesToReceive];
	readChars(receiveBuffer, bytesToReceive);	

	switch(question)
	{
		case 0: 
			writeString_P("Hello \"");
			writeStringLength(&receiveBuffer[0],bytesToReceive,0);
			writeString_P("\" !\n");
		break;
		case 1: 
			writeString_P("So your age is: \""); 
			writeStringLength(&receiveBuffer[0],bytesToReceive,0);
			writeString_P("\" ! Guess what? Nobody cares... ;-)\n");
		break;
		case 2:
			if(receiveBuffer[0] == 'y') 
			{
				writeString_P("Here we go!\n");
				outputTextStuff();
				writeString_P("\n\nThat's it!\n\n"); 
			}
			else
				writeString_P("OK then... let's move on!\n"); 
		break;
		case 3:
			if(receiveBuffer[0] == 'y')
			{
				writeString_P("Now you get a led and beeper show!\n");
				uint8_t i = 0;
				for(;i < 10; i++)
					Leds_Beeper();
				writeString_P("Oh ... it's over!\n");
			}
			else
				writeString_P("Well, if you don't want that... :*( \n"); 
		break;
		case 4:
			if(receiveBuffer[0] == 'y')
			{
				writeString_P("Great! Here we go again...\n"); 
				question = -1;
			}
			else
			{
				writeString_P("OK good bye! I will wait \n");
				writeString_P("until someone resets me!\n"); 
				question = -2;
			}
		break;
	}
	
	question++;
}

int main(void)
{
	question = 0;
	
	initRobotBase(); // Always call this first! The Processor will not work
					 // correctly otherwise.
	
	Leds_Beeper();
	
	writeString_P("\nHello! My name is RobotArmy\n");
	
	while(true)
	{
	if(question != -1)
			askAQuestion();
	
	}
	return 0;
}
