/* 
 * ****************************************************************************
 * Robotarm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm I2C
 * Author(s): Hein Wielink
 			  Huy Nguyen 	
 * ****************************************************************************
 * Description:
 * In this examle the display of Yeti will display a '0' at the begin. 
 * Every second the value will be increment with one.
 * (For the use of this program you need a yeti display)
 * ****************************************************************************
 */

/*****************************************************************************/
// Includes:

#include <RobotArmBaseLib.h>
#include <I2C_Yeti_Display.h>
 
int main(void)
{

	int i = 0;
	int timer;
	
	//Init Robotarm 
	initRobotBase();
	//Init Display
	vInitDisplay();
	
	//Set Robotarm powerLED on green
	PowerLEDorange();
	
	while(1){
		
		//Write value 'i' to display
		ucShowPositiveDecimalValueInDisplay(i);
		
		//Wait 10 times 10 ms = 1 sec
		for(timer=0; timer<10; timer++){
			mSleep(100);
		}
		
		//increment value 'i' with 1
		i++;
	
	}
	
	
}
	