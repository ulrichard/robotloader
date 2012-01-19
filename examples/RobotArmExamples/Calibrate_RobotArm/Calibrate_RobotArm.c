/* 
 * ****************************************************************************
 * Robotarm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm calibration
 * Author(s): Hein Wielink
			  Huy Nguyen 
 * ****************************************************************************
 * Description:
 * In this Example program the servomotors will be calibrated.
 * This is the first program you have to use before you create your own programs.
 * Without the use of this calibration program you can damage your Robotarm!
 *
 * ****************************************************************************
 */

/*****************************************************************************/
// Includes:

#include "RobotArmBaseLib.h"// The Robotarm Robot Library.

char receiveBuffer[UART_RECEIVE_BUFFER_SIZE];

void calibration(void)
{	
	Power_Servos();  //Use this function to power the servo motors on
					 //When you want to power off the servos, you need to call function Power_Off_Servos();

	//defines:
	char dir = 0;	
	int CalServo = 1;
	signed int position = 0;

	// ---------------------------------------
	// Write messages to the Serial Interface
	writeString_P("#### Calibration #####\n");
	writeString_P("To calibrate the robot arm in the right position, you have to calibrate \n");
	writeString_P("the robot arm in the same position as the picture in the manual\n");
	writeString_P("(chapter: calibration)\n\n");	
	writeString_P("You can use the next keys to change the position:\n");
	writeString_P("Enter h:  Stop the servo movement (hold)\n");
	writeString_P("Enter p:  Servo position +  \n");
	writeString_P("Enter m:  Servo position - \n");
	writeString_P("Enter ok: If the servo stands in the correct position,\n");
	writeString_P("Enter x:  It will stop de calibration program\n");
	writeString_P(" \n\n");

	receiveBytes(2);
	writeString_P("#### Calibrate Servo 1 #####\n");
	writeString_P("Please enter your choice: h,p,m,ok,x\n");
	
	
	Start_Position[1] = 1500; 	

while(CalServo<7)
	{
		if(getUARTReceiveStatus() != UART_BUISY)
		{
			copyReceivedBytesToBuffer(&receiveBuffer[0]);
				
		
				
			if(receiveBuffer[0]=='o' && receiveBuffer[1]=='k' ) //Input 'ok'? Centre servo position
			{
				Start_Position[CalServo] = position + 1500;
				
				writeString_P("Servo ");
				writeInteger(CalServo,DEC);
				writeString_P(" default value is: ");
				writeInteger(Start_Position[CalServo],DEC);
				writeString_P("\n\n");
				
				CalServo ++;
				
				if(CalServo<7)
				{
					Start_Position[CalServo] = 1500; 	
					position = 0;
					dir=0;
					writeString_P("#### Calibrate Servo ");
					writeInteger(CalServo, DEC);
					writeString_P(" ####\n");
					writeString_P("Please enter your choice: h,p,m,ok or x\n");
				}
			}

			else if(receiveBuffer[0]=='x') //Input 'x'? Stop program
			{
			    writeString_P("END\n");
				break;				
			}

			else if(receiveBuffer[0]=='h') //Input 'h'? Hold servo position
			{
				dir = 0;
			    writeString_P("Stop (hold) \n");
			}

			else if (receiveBuffer[0]=='p') //Input 'p'? Servo position +
			{
			    writeString_P("Plus\n");
				dir = 1;
			}

			else if (receiveBuffer[0]=='m') //Input 'm'? Servo position -
			{
			    writeString_P("Minus\n");
				dir = 2; 
			}

			else //Input else? Ignore and return "incorrect command"
			{
			writeString_P("incorrect command\n");
			}

			receiveBytes(2);
		}
		
		if (dir==1)
			{
			position+=1;
			}
		
		else if(dir==2)  
			{
			position-=1;
			}
				
			Move(CalServo, position);
			mSleep(15);
			
		if(getStopwatch1() > 1000 && dir!= 0) // have we reached AT LEAST 1000ms = 1s?
				{
				writeString_P("Position: ");
				writeInteger( ( position + 1500 ),DEC);
				writeString_P("\n");
				setStopwatch1(0);  
				}
			
	}


	
	for (int j=1;j<CalServo;j++) //Display servo centre position values
	{
			
	writeString_P("Default value for servo ");
	writeInteger(j,DEC);
	writeString_P(" = ");
	writeInteger(Start_Position[j],DEC);
	writeString_P("\n");
	}

	writeString_P("\nWrite values to EEprom   ...   ");
	
	write_Values_EE();		// To write EEPROM from SRAM
	
	writeString_P("Succes!\n");
	
	Power_Off_Servos();
	
	writeString_P("Please press 'x' to restart\n");
	
	receiveBytesToBuffer(1, &receiveBuffer[0]);
}




int main(void)
{   

	initRobotBase(); // Always call this first! The Processor will not work
					 // correctly otherwise.
	
	startStopwatch1();
	
	while(true)
	{
		calibration(); //Function to calibrate the servomotors
	}


	// End of main loop
	// ---------------------------------------

	// ---------------------------------------
	// The Program will NEVER get here!
	// (at least if you don't perform a "break" in the main loop, which
	// you should not do usually!)

	return 0; 

	
}
