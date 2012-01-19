/* 
 * ****************************************************************************
 * Robotarm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm keyboard
 * Author(s): Hein wielink
			  Huy Nguyen 
 * ****************************************************************************
 * Description:
 * In this program you can control the Robotarm with an external keyboard
 *
 * ############################################################################
 * ATTENTION: Make sure you have calibrated the Robotarm or else you
 * can damage de servos!!!
 * You can calibrate the servos with the selftest program. (press c)
 * ############################################################################
 * ****************************************************************************
 */
 
/*****************************************************************************/
// Includes:

#include "RobotArmBaseLib.h"// The Robotarm Robot Library.
							// Always needs to be included!

int speed;

void Event( int x ){

	switch(x){
	
		case 0: { writeString_P("\n Nothing "); break;}
		case 1: { Pos_Servo_1--; mSleep(speed); break;}
		case 2: { Pos_Servo_1++; mSleep(speed); break;}
		case 3: { Pos_Servo_2--; mSleep(speed); break;}
		case 4: { Pos_Servo_2++; mSleep(speed); break;}
		case 5: { Pos_Servo_3--; mSleep(speed); break;}
		case 6: { Pos_Servo_3++; mSleep(speed); break;}
		case 7: { Pos_Servo_4--; mSleep(speed); break;}
		case 8: { Pos_Servo_4++; mSleep(speed); break;}
		case 9: { Pos_Servo_5--; mSleep(speed);break;}
		case 10: {  Pos_Servo_5++; mSleep(speed);break;}
		case 11: {  Pos_Servo_6--; mSleep(speed);break;}
		case 12: {  Pos_Servo_6++; mSleep(speed);break;}
		case 13: { writeString_P("\n TANK FNT"); break;}
		case 14: { writeString_P("\n TANK BCK"); break;}
		case 15: { writeString_P("\n TANK RIGHT"); break;}
		case 16: { writeString_P("\n TANK LEFT"); break;}
		
	}

}
/*****************************************************************************/
// Main function - The program starts here:

int main(void)
{
	initRobotBase(); 	// Always call this first! The Processor will not work
						// correctly otherwise.		 
	mSleep(1000);        // delay 1s

	speed = 0; //Speed for servo from '0'(fast) - '10'(slow)

	writeString_P("\n Key Board Control \n\n");

	Start_position(); //Use this function to set the servomotor in the centre. 
					  //This function must be called before using Power_Servos();
	
	Power_Servos();  //Use this function to power the servo motors on
					 //When you want to power off the servos, you need to call function Power_Off_Servos();
	
	
	// ---------------------------------------
	// Main loop:
	while(true)
	{
		
		Event(scan_keyboard());
		
	// End of main loop!
	// ---------------------------------------
	}
	return 0;
}
