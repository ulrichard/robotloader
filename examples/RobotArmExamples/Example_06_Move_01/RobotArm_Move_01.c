/* 
 * ****************************************************************************
 * Robotarm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm move example 1
 * Author(s): Hein wielink
			  Huy Nguyen 
 * ****************************************************************************
 * Description:
 * In this example you can see how you can move the Robotarm.
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

#include "RobotArmBaseLib.h"

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
	
	while(true)
	{
		// "s_Move( a, b, c );" Function to move the servo
		// The function has 3 variables:
		// 		a - Which servo 	(0,1,2,3,4,5 or 6) 
		//		b - The position 	(Min. -500 ..... +500 max) 
		//		c - The speed 		(Fast 0  ......  10 slow) 
		
		// Example:
		// s_Move( a,   b, c );
		// s_Move( 6, 500, 2 );
		//		a - Servo 6
		//		b - Max postion of servo 6
		//		c - Speed 2
		
		mSleep(800);
		s_Move(5, 800,3);	
		s_Move(1, -500,1);	
		s_Move(1, 0,1);	
		s_Move(2, -500,1);	
		s_Move(2, 500,1);	
		s_Move(2, 0,1);			
		s_Move(3, 400,1);		
		s_Move(3, -500,1);		
		s_Move(3, 0,1);				
		s_Move(4, 500,2);			
		s_Move(4, -400,2);		
		s_Move(4, 0,2);	
		s_Move(6, 500,2);			
		s_Move(6, -500,2);		
		s_Move(6, 0,2);	
							

	}
	
	
	// End of main loop!
	// ---------------------------------------

	return 0;

}
