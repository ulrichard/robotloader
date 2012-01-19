/* 
 * ****************************************************************************
 * Robotarm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm led and beeper 
 * Author(s): Hein wielink
			  Huy Nguyen 
 * ****************************************************************************
 * Description:
 * A typical "Hello World" program for the Robotarm. Writes text to the PC with the
 * Serial Interface and show a blinking led and beep sound.
 *
 * You can watch the text output when robotarm is connected to the USB interface
 * with the Terminal in RobotLoader!
 * In the RobotLoader Software switch to the tab "Terminal" and use the
 * menu item RobotLoader->Start Target or whatever the text is for the 
 * language you selected. (or press strg+s on your keyboard)
 *
 * ############################################################################
 * The Robot does NOT move in this example! You can simply put it on a table
 * next to your PC and you should connect it to the PC via the USB Interface!
 * ############################################################################
 */
 
/*****************************************************************************/
// Includes:

#include "RobotArmBaseLib.h"// The Robotarm Robot Library.
								// Always needs to be included!

/*****************************************************************************/
// Main function - The program starts here:

int main(void)
{
	initRobotBase(); // Always call this first! The Processor will not work
					 // correctly otherwise.		 
	
	// ---------------------------------------
	// Write messages to the Serial Interface
	writeString_P("\n\n   _______________________\n");
	writeString_P("   \\|   ROBOT SYSTEM   |/\n");
	writeString_P("    \\_-_-_-_-_-_-_-_-_-_/\n\n");

	// Explanation of special chars:
	// '\n' = new line
	// '\\' = '\'
	// These are "escape sequences" for characters you can not
	// use directly within strings.

	// Write "Hello World" to the Serial Interface:
	writeString_P("Hello World!!!\n");
	writeString_P("Let's go! :)\n");

	// ---------------------------------------
	// Main loop - the program will loop here forever!
	while(true)
	{

		// ---------------------------------------
		// Led and Beeper:
	
		changeBeepsound(255);	// Change beepsound
		setBeepsound();			// Beeper on 
		PowerLEDred();			// Power led = red 
		mSleep(500); 			// delay 500ms = 0.5s
	
		changeBeepsound(200);	// Change beepsound 
		PowerLEDgreen();		// Power led = green 
		mSleep(500); 			// delay 500ms = 0.5s
	
		changeBeepsound(150);	// Change beepsound 
		PowerLEDorange();		// Power led = orange (green and red)
		mSleep(500); 			// delay 500ms = 0.5s
	
		PowerLEDoff();			// Power led off 
		clearBeepsound();		// Beeper off 
		mSleep(500); 			// delay 500ms = 0.5s	
	
	
	}
	// End of main loop!
	// ---------------------------------------

	return 0;
}
