/* 
 * ****************************************************************************
 * Robotarm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm current measurments of servo motors
 * Author(s): Hein wielink
			  Huy Nguyen 
 * ****************************************************************************
 * Description: 
 * In this program you can see the current usage of your robotarm.
 * The actual current will be displayed in terminal, when the current
 * is too high the program disconnect the power of the servos. 
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

/*****************************************************************************/

char receiveBuffer[UART_RECEIVE_BUFFER_SIZE];
int i=1;
	
void highCurrent( int servo, int currentvalue, int max_current ){
	
	writeString_P("\n\nWARNING: SERVO");
	writeIntegerLength(servo, DEC, 1);
	writeString_P(" CURRENT TOO HIGH!\n");
	writeString_P("Read current = ");
	writeIntegerLength(currentvalue, DEC, 3);
	writeString_P("\n Max current = ");
	writeIntegerLength(max_current, DEC, 3);

	writeString_P("\n\nPress 'x' to restart program\n");
	Power_Off_Servos(); 
	i=0; //Stop program

}	

// Main function - The program starts here:

int main(void)
{

	initRobotBase(); // Always call this first! The Processor will not work
					 // correctly otherwise.	
	
	Start_position(); //Use this function to set the servomotor in the centre. 
					  //This function must be called before using Power_Servos();
					  
	Power_Servos();  //Use this function to power the servo motors on.
					 //When you want to power off the servos, you need to call function Power_Off_Servos();
	
	writeString_P("########################################\n");
	writeString_P("########   Measure Currents  ###########\n\n"); 
	
	mSleep(1000);

	while (true)
	{
		if(i){
			Current_1 = readADC(ADC_CURRENT_1);  //Read current of servo 1
			writeString_P("Cur 1: ");
			writeIntegerLength(Current_1, DEC, 3); //Display current value servo 1
		
			Current_2 = readADC(ADC_CURRENT_2); //Read current of servo 2
			writeString_P("  |Cur 2: ");
			writeIntegerLength(Current_2, DEC, 3); //Display current value servo 2

			Current_3 = readADC(ADC_CURRENT_3); //Read current of servo 3
			writeString_P("  |Cur 3: ");
			writeIntegerLength(Current_3, DEC, 3); //Display current value servo 3

			Current_4 = readADC(ADC_CURRENT_4); //Read current of servo 4
			writeString_P("  |Cur 4: ");
			writeIntegerLength(Current_4, DEC, 3); //Display current value servo 4

			Current_5 = readADC(ADC_CURRENT_5); //Read current of servo 5
			writeString_P("  |Cur 5: ");
			writeIntegerLength(Current_5, DEC, 3);	//Display current value servo 5
		
			Current_6 = readADC(ADC_CURRENT_6); //Read current of servo 6
			writeString_P("  |Cur 6: ");
			writeIntegerLength(Current_6, DEC, 3);	//Display current value servo 6	

			writeChar('\n'); 
		
			mSleep(500); 
			
			if(Current_1 > max_current_servo1){ highCurrent( 1,Current_1,max_current_servo1 );	} //Is the current 1 higher than max current 1?
			if(Current_2 > max_current_servo2){ highCurrent( 2,Current_2,max_current_servo2 );	} //Is the current 2 higher than max current 2?
			if(Current_3 > max_current_servo3){ highCurrent( 3,Current_3,max_current_servo3 );	} //Is the current 3 higher than max current 3?
			if(Current_4 > max_current_servo4){ highCurrent( 4,Current_4,max_current_servo4 );	} //Is the current 4 higher than max current 4?
			if(Current_5 > max_current_servo5){ highCurrent( 5,Current_5,max_current_servo5 );	} //Is the current 5 higher than max current 5?
			if(Current_6 > max_current_servo6){ highCurrent( 6,Current_6,max_current_servo6 );	} //Is the current 6 higher than max current 6?
			
			if(i == 0){ //Wait for 'x' input to restart program
				receiveBytesToBuffer(1, &receiveBuffer[0]);
				if(receiveBuffer[0] == 'x')	{
					writeString_P("########################################\n");
					writeString_P("######  RESTART Measure Currents  ######\n\n");
					Start_position();
					Power_Servos();
					mSleep(500); 
					i=1;
				}
			}
		}	
	}	

	// End of main loop!
	// ---------------------------------------

	return 0;
}
