/* 
 * ****************************************************************************
 * Robotarm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm uart example 1
 * Author(s): Hein wielink
			  Huy Nguyen 
 * ****************************************************************************
 * Description:
 * In this example the uart of the Robotarm will be tested by displaying several
 * numbers in different types. BIN, OCT, DEC and HEX. 
 *
 * ############################################################################
 * The Robot does NOT move in this example! You can simply put it on a table
 * next to your PC and you should connect it to the PC via the USB Interface!
 * ############################################################################
 * ****************************************************************************
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
	
	// Write a text message to the UART:
	writeString_P("\nJust a simple counter program \n\n");

	// Define a counting variable:
	uint16_t counter = 0;

	// ---------------------------------------
	// Main loop:
	while(true)
	{
		// Now we check what value the counter has, is the value...
		if(counter < 100) // ... is it smaller than 100?
		{
			// True --> output the Counter value with the "writeInteger"
			// function:
			writeString_P("Counter: ");
			writeInteger(counter, BIN);
			writeString_P("(BIN) | ");
			writeInteger(counter, OCT);
			writeString_P("(OCT) | ");
			writeInteger(counter, DEC);
			writeString_P("(DEC) | ");
			writeInteger(counter, HEX);
			writeString_P("(HEX) ");
		}
		else 			  // ... or is the value greater than or equal to 100?
		{
			// False, the counter >= 100 --> use "writeIntegerLength" instead.
			writeString_P("Counter: ");
			writeIntegerLength(counter, BIN, 16);  
			writeString_P("(BIN) | ");
			writeIntegerLength(counter, OCT, 6);
			writeString_P("(OCT) | ");
			writeIntegerLength(counter, DEC, 6);
			writeString_P("(DEC) | ");
			writeIntegerLength(counter, HEX, 4);
			writeString_P("(HEX) ");
		}
		
		writeChar('\n'); // New Line
		
		counter++;    // Increment counter
		
		mSleep(100); // delay 100ms = 0.1s
	}
	// End of main loop!
	// ---------------------------------------

	return 0;
}
