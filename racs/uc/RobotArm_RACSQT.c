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

int main(void)
{
	initRobotBase(); // Always call this first! The Processor will not work
					 // correctly otherwise.	

	Start_position(); //Use this function to set the servomotor in the centre. 
					  //This function must be called before using Power_Servos();
	
	Power_Servos();  //Use this function to power the servo motors on
					 //When you want to power off the servos, you need to call function Power_Off_Servos();
					 
	mSleep(1000); 	 

	static const char fmtmsg[] = "\nThe format is: \"i:pnnn;\" where i is the servo number [1-6], p is an optional sign [+-] and nnn is the 3 digit target position\nOr \"s:nn\" for setting the servo speed, where s is a literal s character and nn is the two digit speed where 00 is fast and 10 is slow.\n\n";
	
	// Write a text message to the UART:
	writeString_P("\nracsqt ready to receive commands.");
	writeNStringP(fmtmsg);

	char    recbuf[16];
	uint8_t bufpos = 0;
	uint8_t servospeed = 3;	// The speed (Fast 0  ......  10 slow) 
	// ---------------------------------------
	// Main loop:
	while(true)
	{
		char cc;
		receiveBytesToBuffer(1, &cc);

		recbuf[bufpos++] = cc;

		if(bufpos < 6 || (bufpos == 6 && isdigit(cc)))
			continue;

		if(bufpos == 4 && (recbuf[0] == 's' || recbuf[0] == 'S'))
		{
			recbuf[6] = '\0';
			servospeed = atoi(&recbuf[2]);
			bufpos = 0;
			continue;
		}

		if(!isdigit(recbuf[0]) || recbuf[1] != ':' || (!isdigit(recbuf[2]) && recbuf[2] != '-' && recbuf[2] != '+') || !isdigit(recbuf[3]) || !isdigit(recbuf[4]) || (bufpos > 5 && !isdigit(recbuf[5])))
		{
			if(bufpos > 2)
				writeNStringP(fmtmsg);
			bufpos = 0;
			continue;
		}

		const uint8_t servonum = recbuf[0] - '0'; // Which servo 	(0,1,2,3,4,5 or 6) 
		recbuf[6] = '\0';
		const int16_t servopos = atoi(&recbuf[2]); // The position 	(Min. -500 ..... +500 max) 

		s_Move(servonum, servopos, servospeed);
	}
	// End of main loop!
	// ---------------------------------------

	return 0;
}
