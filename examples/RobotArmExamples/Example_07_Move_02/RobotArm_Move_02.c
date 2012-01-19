/* 
 * ****************************************************************************
 * Robotarm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm move example 2
 * Author(s): Hein wielink
			  Huy Nguyen 
 * ****************************************************************************
 * Description:
 * In this example a pick up and place program will be demonstrated. 
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
		
	s_Move(1, -500,2);	//Gripper open
	
		//Pick Up
		// -Down
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
		
			s_Move(3, -500,2);
			s_Move(4, -450,2);
			s_Move(5, 800,2);	
			s_Move(1, 0,2);
		
			// -up
			s_Move(5, 700,4);			
			s_Move(3, -100,2);		
			s_Move(4, 0,2);			
	
	
			//Place
			// Down 
			s_Move(6, 500,2);
			s_Move(4, -500,2);
			s_Move(3, -450,2);
			s_Move(5, 800,4);
			s_Move(1, -500,2);
			
			//Place
			// up
			s_Move(5, 700,4);			
			s_Move(3, -100,2);	
			s_Move(4, 0,2);	
			s_Move(6, 0,2);
		}
	
	// End of main loop!
	// ---------------------------------------

	return 0;
}
