/* 
 * ****************************************************************************
 * RobotArm - ROBOT BASE EXAMPLES
 * ****************************************************************************
 * Example: Robotarm keyboard
 * Author(s): Hein wielink
			  Huy Nguyen 
 * ****************************************************************************
 * Description:
 * In this program you can make music with the external keyboard!!!
 * ****************************************************************************
 */
/*****************************************************************************/
// Includes:

#include "RobotArmBaseLib.h"// The RobotArm Robot Library.
								// Always needs to be included!

#define Note_C	130
#define Note_D	155
#define Note_E	165
#define Note_F	175
#define Note_G	200
#define	Note_A	220

void Music_note( int x ){

	switch(x){
	
		case 0: {  clearBeepsound(); break;}
		case 1: {  changeBeepsound(Note_C); setBeepsound(); break;}
		case 2: {  break;}
		case 3: {  changeBeepsound(Note_D); setBeepsound(); break;}
		case 4: {  break;}
		case 5: {  changeBeepsound(Note_E); setBeepsound(); break;}
		case 6: {  break;}
		case 7: {  changeBeepsound(Note_F); setBeepsound();break;}
		case 8: {  break;}
		case 9: {  changeBeepsound(Note_G); setBeepsound(); break;}
		case 10: {  break;}
		case 11: {  changeBeepsound(Note_A); setBeepsound(); break;}
		case 12: {  break;}
		case 13: {  break;}
		case 14: {  break;}
		case 15: {  break;}
		case 16: {  break;}
		
	}
}

/*****************************************************************************/
// Main function - The program starts here:

int main(void)
{
	initRobotBase(); // Always call this first! The Processor will not work
					 // correctly otherwise.		 
	
	// Write a text message to the UART:
	writeString_P("\nMusic with keyboard\n\n");

	// ---------------------------------------
	// Main loop:
	while(true)
	{
	
		Music_note(scan_keyboard());
		
	}
	// End of main loop!
	// ---------------------------------------

	return 0;
}
