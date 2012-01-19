/* 
 * ****************************************************************************
 * Robotarm - ROBOT EXAMPLES
 * ****************************************************************************
 * Example: Robotarm selftest 
 * Author(s): Hein Wielink
 *	          Huy Nguyen 
 * ****************************************************************************
 * Description:
 *
 * Test routines for all robotarm components. 
 *
 * Yes we know - this program has not the most beautiful code on earth... 
 * To be honest it is quite ugly ;) 
 * All the Text output is just a waste of program space and this is 
 * intentionally to get a large program ;) 
 *
 * It is a good demonstration how big your programs can get! 
 * Consider that every single character of a String consumes one byte 
 * of program space!
 * Also look at it in the hexfile viewer in Robot Loader - nearly half the 
 * program memory is full with ASCII Text. *  
 *
 * #+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+#+
 * ############################################################################
 * ****************************************************************************
 */
 
/*****************************************************************************/
// Includes:

#include "RobotArmBaseLib.h"

/*****************************************************************************/

char receiveBuffer[UART_RECEIVE_BUFFER_SIZE];

void done(void)
{
	writeString_P("Done!\n"); 
}

void bars(uint8_t number)
{
	uint8_t cnt;
	writeChar('\n');
	for(;number > 0; writeChar('\n'), number--)
		for(cnt = 69; cnt ; cnt--)
			writeChar('#');
}


void test(uint8_t number)
{
	bars(2);
	writeString_P("#### TEST #");
	writeInteger(number, DEC);
	writeString_P(" ####\n");
}

void printUBat(uint16_t ubat)
{
	writeIntegerLength((((ubat/204.8f)+0.1f)), DEC, 2);
	writeChar('.');
	writeIntegerLength((((ubat/2.048f)+10)), DEC, 2);
	writeChar('V');
}

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

void StatusLedTest(void)
{	
	test(1);
	writeString_P("\n### POWER ON TEST ###\n");
	writeString_P("Please watch the PowerLED and verify that it lights up!\n");
	writeString_P("(The color of the PowerLED will change:)\n");
	PowerLEDred(); 
	mSleep(2000);
	PowerLEDgreen();
	mSleep(2000);
	PowerLEDorange();
	mSleep(2000);
	PowerLEDoff();
	done();
}


/*****************************************************************************/
//

void VoltageTest(void)
{
	test(2);
	writeString_P("\n### Voltage Sensor Test ###\n");
	writeString_P("\n Press ENTER to start battery measurements!\n\n");
    receiveBytesToBuffer(1, &receiveBuffer[0]);

	writeString_P("Performing 10 measurements:\n");
	uint16_t ubat;
	uint8_t i;
	for(i = 1; i <= 10; i++)
	{
		writeString_P("Measurement #"); 
		writeInteger(i, DEC);
		writeString_P(": ");
		ubat = readADC(ADC_UBAT);
		printUBat(ubat);

		if(ubat >= 650 && ubat <= 1020)
		{
			writeString_P(" --> OK!\n");	
		}
		
		else 
		{
			if(ubat < 650) 
			{
				writeString_P("WARNING: VOLTAGE IS TOO LOW\n");
			}
			else if(ubat > 1020)
			{
				writeString_P(" --> EXCELLENT!\n!");
			}
		}
		
		mSleep(50);
	
	}
	mSleep(500); 
	done();
}



/*****************************************************************************/

void BuzzerTest(void)
{	
	test(3);
	writeString_P("Buzzer Test\n");
	writeString_P("Please listen to the Buzzer and verify!\n");
	writeString_P("The Buzzer will peep a few times!\n");
	writeString_P("### The test is running now! ### \n\n");
	
	setBeepsound();
	mSleep(200);
	clearBeepsound();
	mSleep(600);
	setBeepsound();
	mSleep(400);
	clearBeepsound();
	mSleep(600);
	setBeepsound();
	mSleep(800);
	clearBeepsound();
	mSleep(600);
	
	done();
}


void ServoTest(void)
{
	test(4);
	writeString_P("\nAutomatic Servomotor test\n\n");
	writeString_P("###############################################\n");
	writeString_P("### ATTENTION!!! DANGER!!! WARNING!!!\n");
	writeString_P("Make sure the servomotors can move FREE! DO NOT BLOCK THEM!\n");
	writeString_P("###############################################\n");
	writeString_P("\n Press ENTER to start servo test!\n");
    receiveBytesToBuffer(1, &receiveBuffer[0]);

	writeString_P("Set Start Position: \n");
	Start_position();

	Power_Servos ();
	uint16_t pos;
	
	writeString_P("Servo: 1 moves\n");
	Display_Current();
	s_Move(1, -400,2);	
	Display_Current();
	s_Move(1, 0,2);	
	Display_Current();
	mSleep(200);
			
	for (int servo = 2; servo < 7 ; servo++)
	{ 
		writeString_P("Servo: ");
		writeInteger(servo,DEC);
		writeString_P(" moves\n");
		pos = Start_Position[servo];
		Display_Current();
		s_Move(servo, 500,2);
		Display_Current();
		s_Move(servo, -500,2);	
		Display_Current();
		s_Move(servo, 0,2);	
	}
	
	
	
	Power_Off_Servos();
	
	done();
}


/*****************************************************************************/
// 

void calibration(void)
{
	Power_Servos();
	startStopwatch1();
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
				
			
				
			if(receiveBuffer[0]=='o' && receiveBuffer[1]=='k' )
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

			else if(receiveBuffer[0]=='x')
			{
			    writeString_P("END\n");
				break;				
			}

			else if(receiveBuffer[0]=='h')
			{
				dir = 0;
			    writeString_P("Stop (hold) \n");
			}

			else if (receiveBuffer[0]=='p')
			{
			    writeString_P("Plus\n");
				dir = 1;
			}

			else if (receiveBuffer[0]=='m')
			{
			    writeString_P("Minus\n");
				dir = 2; 
			}

			else 
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


	
	for (int j=1;j<CalServo;j++)
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
	stopStopwatch1();
	
	writeString_P("Press ENTER to return to main screen\n");
	
	receiveBytesToBuffer(1, &receiveBuffer[0]);
}

//Current measuring:

void current_measuring (void)
{
	bars(2);
	writeString_P("#### Current Measuring #####\n");

	writeString_P("Press ENTER to return to main screen!\n\n");
	writeString_P("### The test is running now! ### \n\n");
	
	receiveBytes(1);
	
	Start_position();
	Power_Servos();

	while (true)
	{
			if(getUARTReceiveStatus() != UART_BUISY)
			{	
				copyReceivedBytesToBuffer(&receiveBuffer[0]);
				break;
			}
			
			Display_Current(); 
			mSleep(500); 
	
	}

	Power_Off_Servos();	
	
	writeString_P(" #### End #####\n\n");
	
	
}

//Start position 

void Set_Start_position (void)
{
	bars(2);
	writeString_P("#### Startposition Robotarm#####\n");

	Start_position();
    Power_Servos();

	mSleep(3000);
	
	Power_Off_Servos();
	done();
}



/*****************************************************************************/
// Main:


int main (void)
{
	uint8_t test = 0;
	initRobotBase();
			
	writeString_P("\n Robotarm Selftest\n");
	writeString_P("#####################################################################\n");
	writeString_P("### ATTENTION!!! DANGER!!! WARNING!!!\n");
	writeString_P("\n");
	writeString_P("First use Robotarm! \n");
	writeString_P("1) Type 'C' in main menu of selftest to calibrate robotarm\n");
	writeString_P("2) Calibrate the robotarm servomotors \n");
	writeString_P("\n");
	writeString_P("#####################################################################\n\n");
	writeString_P("Press ENTER to continue!\n"); 

	receiveBytesToBuffer(1, &receiveBuffer[0]);
	
	while(1)
	{
		// This menu is mainly a program space filler - good for serial interface (speed) testing ;)
		// The same applies for ALL other text information in this program!
		// All this is just to have a BIG program download ;)
		writeChar('\n');
		writeString_P("#####################################################################\n"); 
		writeString_P("#########               Robotarm Selftest                   #########\n"); 
	    writeString_P("#####################################################################\n"); 
		writeString_P("#####       Main Menu         #########      Advanced Menu      #####\n"); 
		writeString_P("#                                 #                                 #\n"); 
		writeString_P("# 0 - Run ALL Selftests (1-4)     # c - Calibrate Robotarm          #\n"); 
		writeString_P("# 1 - LED Test                    # m - Current Measuring Servos    #\n"); 
		writeString_P("# 2 - Voltage Sensor Test         # b - Start Position              #\n"); 
		writeString_P("# 3 - Buzzer Test                 #                                 #\n"); 
		writeString_P("# 4 - Servo Test                  # System voltage is: ");
		printUBat(readADC(ADC_UBAT));						 
		writeString_P("       #\n"); 							
		writeString_P("#                                 #                                 #\n"); 
		writeString_P("#####################################################################\n"); 
		writeString_P("# Please enter your choice (0-4, c, m or b)!                        #\n");
		writeString_P("#####################################################################\n"); 
		
		receiveBytesToBuffer(1, &receiveBuffer[0]);
		
		test = receiveBuffer[0] - 48;
		
		if(receiveBuffer[0] == 'c')
		{
			// Calibrate the Robotarm
			calibration();
		}
		
		if(receiveBuffer[0] == 'm')
		{
			// Current Measuring 
			current_measuring();
		}
		
				

		if(receiveBuffer[0] == 'b')
		{
			// Robotarm in startposition
			Set_Start_position();
		}
		
		
		
		
		
		else if(test > 8)
		{
			writeString_P("You need to enter a single number from 0 to 4, c, m or b!");
			continue;
		}

		else
		{
			switch(test)
			{
				case 0: StatusLedTest();	
						VoltageTest();
						BuzzerTest();
						ServoTest();
				break; 	
				case 1: StatusLedTest();	break; 
				case 2: VoltageTest();		break; 		
				case 3: BuzzerTest();		break;
				case 4: ServoTest();		break;
			
			}
		}




	}
	
	return 0;

}



