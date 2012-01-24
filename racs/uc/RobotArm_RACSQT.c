/* 
 * ****************************************************************************
 * Robotarm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm racsqt
 * Author(s): Richard Ulrich 
 * ****************************************************************************
 * Description:
 * The robot part of the racsqt
 *
 * ****************************************************************************
 */
 
/*****************************************************************************/
// Includes:

#include "RobotArmBaseLib.h"// The Robotarm Robot Library.
							// Always needs to be included!
#include <ctype.h>
/*****************************************************************************/
// Main function - The program starts here:

void welcomeMsg()
{
	writeString_P("\nThe format is: i:pnnn; where ");
	writeString_P("\n   i is the servo number [1-6]");
	writeString_P("\n   p is an optional sign [+-]");
	writeString_P("\n   nnn is the 3 digit target position (-999 ... +999)");
	writeString_P("\nOr s:nn for setting the servo speed, where s is a literal s character");
	writeString_P("\n   and nn is the two digit speed where 00 is fast and 10 is slow. default is 3.\n\n");
}

int main(void)
{
	initRobotBase(); // Always call this first! The Processor will not work
					 // correctly otherwise.	

//	Start_position(); //Use this function to set the servomotor in the centre. 
					  //This function must be called before using Power_Servos();
	
	Power_Servos();  //Use this function to power the servo motors on
					 //When you want to power off the servos, you need to call function Power_Off_Servos();
					 
	mSleep(1000); 	 

	
	// Write a text message to the UART:
	welcomeMsg();
	writeString_P("\nracsqt ready to receive commands.\n\n");

	char    recbuf[16];
	uint8_t bufpos = 0;
	uint8_t servospeed = 0;	// The speed (Fast 0  ......  10 slow) 
	int     verbose = 0;
	// ---------------------------------------
	// Main loop:
	while(true)
	{
		char cc;
		receiveBytesToBuffer(1, &cc);

		recbuf[bufpos++] = cc;
		
		if(bufpos == 1 && !isdigit(recbuf[0]) && recbuf[0] != 's' && recbuf[0] != 'S')
		{
			bufpos = 0;
			continue;
		}

		if(bufpos < 6 || (bufpos == 6 && isdigit(cc)))
			continue;

		if(bufpos == 4 && (recbuf[0] == 's' || recbuf[0] == 'S'))
		{
			recbuf[6] = '\0';
			servospeed = atoi(&recbuf[2]);
			writeString_P("New servo speed: ");
			writeInteger(servospeed, 10);
			writeString_P("\n");
			bufpos = 0;
			continue;
		}

		if(!isdigit(recbuf[0]) || recbuf[1] != ':' || (!isdigit(recbuf[2]) && recbuf[2] != '-' && recbuf[2] != '+') || !isdigit(recbuf[3]) || !isdigit(recbuf[4]) || (bufpos > 5 && !isdigit(recbuf[5])))
		{
			if(bufpos > 2 && verbose)
				welcomeMsg();
			bufpos = 0;
			continue;
		}

		const uint8_t servonum = recbuf[0] - '0'; // Which servo 	(0,1,2,3,4,5 or 6) 
		recbuf[6] = '\0';
		const int16_t servopos = atoi(&recbuf[2]); // The position 	(Min. -500 ..... +500 max) 

		if(verbose)
		{
			writeString_P("Moving servo ");
			writeInteger(servonum, 10);
			writeString_P(" to position ");
			writeInteger(servopos, 10);
			writeString_P("\n");
		}

		if(0 == servospeed)
			Move(servonum, servopos);
		else
			s_Move(servonum, servopos, servospeed);

		bufpos = 0;
		continue;
	}
	// End of main loop!
	// ---------------------------------------
	writeString_P("Fell out of the main loop. Please reset!");

	return 0;
}
