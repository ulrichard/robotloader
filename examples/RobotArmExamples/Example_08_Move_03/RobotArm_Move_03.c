/* 
 * ****************************************************************************
 * Robotarm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm move example 3
 * Author(s): Hein wielink
			  Huy Nguyen 
 * ****************************************************************************
 * Description:
 * In this example the Robotarm will demonstate different movements.
 * In the terminal of the robot loader the current of the servos will be displayed. 
 * When the current is too high, it gives a warning.  
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


//Functions:
void highCurrent( int servo, int currentvalue, int max_current ){
	
	writeString_P("WARNING: SERVO ");
	writeIntegerLength(servo, DEC, 1);
	writeString_P(" CURRENT TOO HIGH!");
	writeString_P("\n");
}

void Display_Current (void)	
	{
			Current_1 = readADC(ADC_CURRENT_1); 
			writeString_P("Cur 1: ");
			writeIntegerLength(Current_1, DEC, 3);
		
			Current_2 = readADC(ADC_CURRENT_2); 
			writeString_P("  |Cur 2: ");
			writeIntegerLength(Current_2, DEC, 3);

			Current_3=readADC(ADC_CURRENT_3); 
			writeString_P("  |Cur 3: ");
			writeIntegerLength(Current_3, DEC, 3);

			Current_4=readADC(ADC_CURRENT_4); 
			writeString_P("  |Cur 4: ");
			writeIntegerLength(Current_4, DEC, 3);

			Current_5=readADC(ADC_CURRENT_5); 
			writeString_P("  |Cur 5: ");
			writeIntegerLength(Current_5, DEC, 3);	
		
			Current_6=readADC(ADC_CURRENT_6); 
			writeString_P("  |Cur 6: ");
			writeIntegerLength(Current_6, DEC, 3);		

			writeChar('\n'); 
		
			if(Current_1 > max_current_servo1){ highCurrent( 1,Current_1,max_current_servo1 );	}
			if(Current_2 > max_current_servo2){ highCurrent( 2,Current_2,max_current_servo2 );	}
			if(Current_3 > max_current_servo3){ highCurrent( 3,Current_3,max_current_servo3 );	}
			if(Current_4 > max_current_servo4){ highCurrent( 4,Current_4,max_current_servo4 );	}
			if(Current_5 > max_current_servo5){ highCurrent( 5,Current_5,max_current_servo5 );	}
			if(Current_6 > max_current_servo6){ highCurrent( 6,Current_6,max_current_servo6 );	}
			
			
	}


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

	while (true)
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
		Display_Current();
		s_Move(5, 700,3);
		Display_Current();	
		s_Move(3, -700,2);	
		Display_Current();
		s_Move(4, -700,2);
		Display_Current();
		
        s_Move(6, 500,1);
		Display_Current();

		s_Move(1, -500,1);
		Display_Current();
		s_Move(1, 0,1);
		Display_Current();
		s_Move(1, -500,1);
		Display_Current();
		s_Move(1, 0,1);		
		Display_Current();

        s_Move(6, 0,1);
		Display_Current();

        s_Move(4, 0,2);
		Display_Current();
        s_Move(3, 0,2);  
		Display_Current();
	
		s_Move(1, -500,1);
		Display_Current();
		s_Move(1, 0,1);
		Display_Current();
		s_Move(1, -500,1);
		Display_Current();
		s_Move(1, 0,1);	
		Display_Current();

					

	}
	
	// End of main loop!
	// ---------------------------------------

	return 0;
}
