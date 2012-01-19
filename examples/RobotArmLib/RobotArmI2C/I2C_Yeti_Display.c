//###########################################################################//
//#                                                                         #//
//#  Filename :  display.c                                                  #//
//#  Project  :  Robotarm                                                   #//
//#                                                                         #//
//#  Description: This file contains all necessary initialisations and      #//
//#  base functions for Yeti display extension PCB                          #//
//#                                                                         #//
//#  (c) Henk van Winkoop (Arexx software development)                      #//
//#                                                                         #//
//#  Version:                                                               #//
//#                                                                         #//
//#  1.00      2006-05-04  Original release                                 #//
//#                                                                         #//
//###########################################################################//

//-------------------------------------------------------------------------------//
//                       YETI DISPLAY EXTENSION KIT                              //
//-------------------------------------------------------------------------------//
//
//  Display numbers
//
//      ---    ---    ---    --- 
//     |   |  |   |  |   |  |   |
//     |   |  |   |  |   |  |   |
//      ---    ---    ---    --- 
//     |   |  |   |  |   |  |   |
//     |   |  |   |  |   |  |   |
//      --- o  --- o  --- o  --- o
//      
//     Disp4  Disp3  Disp2  Disp1     Physical display numbers
//
//       2      4      1      3       Electrical/software display numbering
//
//------------------------------------------------------------
//
//  Single display segment naming
//                
//       a
//     -----
//    |     |
//    |     | b
//  f |     |
//    |  g  |
//     -----
//    |     |
//    |     | c
//  e |     |
//    |     |
//     -----  O dp
//       d    
//
//------------------------------------------------------------
//
//  Bits to control displays as send by TWI TWDR-register to each display.
//              +---+---+---+---+---+---+---+---+
//    segment   | dp| g | f | e | d | c | b | a |
//              +---+---+---+---+---+---+---+---+
//    bit       | 7 | 6 | 5 | 4 | 3 | 2 | 1 | 0 |
//              +---+---+---+---+---+---+---+---+
//
//------------------------------------------------------------

//------------------------------------------------------------
// INCLUDES:

#include "I2C_Yeti_Display.h"


//------------------------------------------------------------
//	DEFINES
//------------------------------------------------------------
//define relation between segment-name and segment-selection-bit
#define SGM_X 0x00	//dummy
#define SGM_A 0x01
#define SGM_B 0x02
#define SGM_C 0x04
#define SGM_D 0x08
#define SGM_E 0x10
#define SGM_F 0x20
#define SGM_G 0x40
#define SGM_P 0x80	//Decimal Point
//define segments needed for constructing hexadecimal numbers
#define DSP_0     SGM_X            |SGM_F|SGM_E|SGM_D|SGM_C|SGM_B|SGM_A 
#define DSP_1     SGM_X                              |SGM_C|SGM_B       
#define DSP_2     SGM_X      |SGM_G      |SGM_E|SGM_D      |SGM_B|SGM_A 
#define DSP_3     SGM_X      |SGM_G            |SGM_D|SGM_C|SGM_B|SGM_A 
#define DSP_4     SGM_X      |SGM_G|SGM_F            |SGM_C|SGM_B       
#define DSP_5     SGM_X      |SGM_G|SGM_F      |SGM_D|SGM_C      |SGM_A 
#define DSP_6     SGM_X      |SGM_G|SGM_F|SGM_E|SGM_D|SGM_C      |SGM_A 
#define DSP_7     SGM_X                              |SGM_C|SGM_B|SGM_A 
#define DSP_8     SGM_X      |SGM_G|SGM_F|SGM_E|SGM_D|SGM_C|SGM_B|SGM_A 
#define DSP_9     SGM_X      |SGM_G|SGM_F      |SGM_D|SGM_C|SGM_B|SGM_A 
#define DSP_A     SGM_X      |SGM_G|SGM_F|SGM_E      |SGM_C|SGM_B|SGM_A 
#define DSP_B     SGM_X      |SGM_G|SGM_F|SGM_E|SGM_D|SGM_C             
#define DSP_C     SGM_X            |SGM_F|SGM_E|SGM_D            |SGM_A 
#define DSP_D     SGM_X      |SGM_G      |SGM_E|SGM_D|SGM_C|SGM_B       
#define DSP_E     SGM_X      |SGM_G|SGM_F|SGM_E|SGM_D            |SGM_A 
#define DSP_F     SGM_X      |SGM_G|SGM_F|SGM_E                  |SGM_A 
//define segments needed for constructing additional characters
#define DSP_DP    SGM_X|SGM_P                                           //Decimal Point
#define DSP_MINUS SGM_X      |SGM_G                                     //Minus
#define DSP_BLANC SGM_X                                                 //Blanc
//define indexes of non-hexadecimal segment codes
#define DSP_DP_INDEX    16
#define DSP_MINUS_INDEX 17
#define DSP_BLANK_INDEX 18
#define DSP_INDEX_COUNT 19	//keep this value always up-to-date (equal to last used index number + 1)
#define NUMOF_SEGMENT_CODE_ARRAY_LOCATIONS DSP_INDEX_COUNT
#define NUMOF_DISPLAYS 4
//define relation between visual display and electrical/software display numbering 
#define DISPLAY1_HARDWARE_DISPLAY_NUMBER 0
#define DISPLAY2_HARDWARE_DISPLAY_NUMBER 2
#define DISPLAY3_HARDWARE_DISPLAY_NUMBER 1
#define DISPLAY4_HARDWARE_DISPLAY_NUMBER 3
//display control byte in display driver chip
#define DCB_DYNAMIC_MODE                       0
#define DCB_DISPLAY1_DISPLAY3_ON               1
#define DCB_DISPLAY2_DISPLAY4_ON               2
#define DCB_ALL_DISPLAY_SEGMENTS_ON            3
#define DCB_ADD_03MA_TO_SEGMENT_OUTPUT_CURRENT 4
#define DCB_ADD_06MA_TO_SEGMENT_OUTPUT_CURRENT 5
#define DCB_ADD_12MA_TO_SEGMENT_OUTPUT_CURRENT 6
//twi addresses
#define SAA1024_WRITE_ADDRESS                  0x70
#define SAA1024_READ_ADDRESS                   0x71



//------------------------------------------------------------
//	GLOBAL VARIABLES
//------------------------------------------------------------
volatile unsigned char gucDisplaySegmentsArray[NUMOF_SEGMENT_CODE_ARRAY_LOCATIONS];

//================================================================================
//	INIT DISPLAY
//================================================================================
void vInitDisplay(void){

	//set slowest clock divider
	TWBR=0x55;

	//set slowest prescaler
	TWSR=0x01;

	//enable internal pullup on SCL and SDA
	PORTD|=(1<<SCL_PC)|(1<<SDA_PC);

	//store hexadecimal segment codes into array
	gucDisplaySegmentsArray[ 0]=DSP_0;
	gucDisplaySegmentsArray[ 1]=DSP_1;
	gucDisplaySegmentsArray[ 2]=DSP_2;
	gucDisplaySegmentsArray[ 3]=DSP_3;
	gucDisplaySegmentsArray[ 4]=DSP_4;
	gucDisplaySegmentsArray[ 5]=DSP_5;
	gucDisplaySegmentsArray[ 6]=DSP_6;
	gucDisplaySegmentsArray[ 7]=DSP_7;
	gucDisplaySegmentsArray[ 8]=DSP_8;
	gucDisplaySegmentsArray[ 9]=DSP_9;
	gucDisplaySegmentsArray[10]=DSP_A;
	gucDisplaySegmentsArray[11]=DSP_B;
	gucDisplaySegmentsArray[12]=DSP_C;
	gucDisplaySegmentsArray[13]=DSP_D;
	gucDisplaySegmentsArray[14]=DSP_E;
	gucDisplaySegmentsArray[15]=DSP_F;
	
	//store additional segment codes into array
	gucDisplaySegmentsArray[16]=DSP_DP;
	gucDisplaySegmentsArray[17]=DSP_MINUS;
	gucDisplaySegmentsArray[18]=DSP_BLANC;
}
//================================================================================
//  Remember:
//   - TWI (Two Wire Interface) is another name for I2C
//   - clearing TWINT bit is done by writing a '1' into it
//   - display driver chip SAA1064 address selection pin is connected to GND
//     this makes chip write address 0x70 and chip read address 0x71 (see datasheet)
//================================================================================
//	SHOW POSITIVE DECIMAL VALUE IN DISPLAY
//	return value: errorcode or '0' if passed
//================================================================================
unsigned char ucShowPositiveDecimalValueInDisplay(unsigned int uiDecVal){
/*
	After a TWSTA command the TWINT flag is only set if a pull-up is applied BOTH
	to SCL and to SDA.
	External pull-up's can be used but also the internal pull-ups will do!
	Using internal pull-up of PD5 in stead of PC5 for SCL also will do.
*/
	//var
	volatile unsigned char ucVal[NUMOF_DISPLAYS];
	volatile unsigned char i;
	volatile unsigned int uiTimOut;

	//----------------------------------------------------------------------//
	// Remember:                                                            //
	// values send to the display:                                          //
	//  - must be segement-codes of the values to be displayed (not the     //
	//    displayed values itself)                                          //
	//  - are retrieved from an predefined array filled with segment-codes  //
	//  - the first 16 locations of the predefined array contain their      //
	//    corresponding segment-codes                                       //
	//    for example:                                                      //
	//     - 'gucDisplaySegmentsArray[ 9]' contains display-segment-code for //
	//       (hexadecimal) character '9'                                    //
	//     - 'gucDisplaySegmentsArray[10]' contains display-segment-code for //
	//       (hexadecimal) character 'A'                                    //
	//----------------------------------------------------------------------//
	
	//if the value is too big to be displayed (more than 4 characters)
	if(uiDecVal>9999){
		//prepair to show a 'minus' sign in all 4 displays
		ucVal[DISPLAY1_HARDWARE_DISPLAY_NUMBER]=DSP_MINUS_INDEX;
		ucVal[DISPLAY2_HARDWARE_DISPLAY_NUMBER]=DSP_MINUS_INDEX;
		ucVal[DISPLAY3_HARDWARE_DISPLAY_NUMBER]=DSP_MINUS_INDEX;
		ucVal[DISPLAY4_HARDWARE_DISPLAY_NUMBER]=DSP_MINUS_INDEX;
	}
	//if the value is small enough to be displayed (4 or less characters)
	else{
		//divide by 1000, remember divider, get remainder
		ucVal[DISPLAY4_HARDWARE_DISPLAY_NUMBER]=uiDecVal/1000; uiDecVal=uiDecVal-ucVal[DISPLAY4_HARDWARE_DISPLAY_NUMBER]*1000;
		//divide by 100,  remember divider, get remainder
		ucVal[DISPLAY3_HARDWARE_DISPLAY_NUMBER]=uiDecVal/100 ; uiDecVal=uiDecVal-ucVal[DISPLAY3_HARDWARE_DISPLAY_NUMBER]*100 ;
		//divide by 10,   remember divider, get remainder
		ucVal[DISPLAY2_HARDWARE_DISPLAY_NUMBER]=uiDecVal/10  ; uiDecVal=uiDecVal-ucVal[DISPLAY2_HARDWARE_DISPLAY_NUMBER]*10  ;
		//get remainder
		ucVal[DISPLAY1_HARDWARE_DISPLAY_NUMBER]=uiDecVal;
	}//else

	//----------------------------//
	//	CREATE 'START' CONDITION	//
	//----------------------------//
	//enable TWI interface, clear TWINT flag, request for START condition, 
	TWCR=(1<<TWEN)|(1<<TWINT)|(1<<TWSTA);
	//wait for 'start condition' being transmitted, return error on timeout
	uiTimOut=10000;
	while(!(TWCR&(1<<TWINT))){
		uiTimOut--;
		if(uiTimOut==0){
			//return error code
			return SC_BUS_ARBITRATION_LOST;
		}
	}
	//if status register does not contain a valid START
	if((TWSR&TWI_STATUS_BITS)!=SC_VALID_SINGLE_START_TX){
		//return expected status as error code
		return SC_VALID_SINGLE_START_TX;
	}
	//----------------------------//
	//	SEND SLAVE ADDRESS				//
	//----------------------------//
	//store (SAA1024) slave address in data register
	TWDR=SAA1024_WRITE_ADDRESS;
	//clear TWINT bit (and keep TWI enabled)
	TWCR=(1<<TWINT)|(1<<TWEN);
	//wait for transmission being executed
	uiTimOut=10000;
	while(!(TWCR&(1<<TWINT))){
		uiTimOut--;
		if(uiTimOut==0){
			//return error code
			return SC_BUS_ARBITRATION_LOST;
		}
	}
	//if status register does not contain valid SLAVE ACKNOWLEDGE
	if((TWSR&TWI_STATUS_BITS)!=SC_VALID_SLAVE_ADDRESS_TX_AND_ACK_RX){
		//return expected status as error code
		return SC_VALID_SLAVE_ADDRESS_TX_AND_ACK_RX;
	}
	//----------------------------//
	//	SEND INSTRUCTION BYTE			//
	//----------------------------//
	//select subaddress 0, so all 4 display values will be send
	TWDR=0x00;	
	//clear TWINT bit (and keep TWI enabled)
	TWCR=(1<<TWINT)|(1<<TWEN);
	//wait for transmission being executed
	uiTimOut=10000;
	while(!(TWCR&(1<<TWINT))){
		uiTimOut--;
		if(uiTimOut==0){
			//return error code
			return SC_BUS_ARBITRATION_LOST;
		}
	}
	//if status register does not contain valid DATA ACKNOWLEDGE
	if((TWSR&TWI_STATUS_BITS)!=SC_VALID_DATA_TX_AND_ACK_RX){
		//return expected status as error code
		return SC_VALID_DATA_TX_AND_ACK_RX;
	}
	//----------------------------//
	//	SEND CONTROL BYTE					//
	//----------------------------//
	//set displays on, use 6mA current, select dynamic mode (needed when using more than 2 displays)
	TWDR=
		 (1<<DCB_ADD_06MA_TO_SEGMENT_OUTPUT_CURRENT)
		|(1<<DCB_DISPLAY2_DISPLAY4_ON)
		|(1<<DCB_DISPLAY1_DISPLAY3_ON)
		|(1<<DCB_DYNAMIC_MODE);
	//clear TWINT bit (and keep TWI enabled)
	TWCR=(1<<TWINT)|(1<<TWEN);
	//wait for transmission being executed
	uiTimOut=10000;
	while(!(TWCR&(1<<TWINT))){
		uiTimOut--;
		if(uiTimOut==0){
			//return error code
			return SC_BUS_ARBITRATION_LOST;
		}
	}
	//if status register does not contain valid DATA ACKNOWLEDGE
	if((TWSR&TWI_STATUS_BITS)!=SC_VALID_DATA_TX_AND_ACK_RX){
		//return error code
		return SC_VALID_DATA_TX_AND_ACK_RX;
	}
	//----------------------------//
	//	SEND DATA BYTES						//
	//----------------------------//
	//handle all displays
	for(i=0;i<NUMOF_DISPLAYS;i++){
		//prepair transmit data
		TWDR=gucDisplaySegmentsArray[ucVal[i]];
		//clear TWINT bit (and keep TWI enabled)
		TWCR=(1<<TWINT)|(1<<TWEN);
		//wait for transmission being executed
		uiTimOut=10000;
		while(!(TWCR&(1<<TWINT))){
			uiTimOut--;
			if(uiTimOut==0){
				//return error code
				return SC_BUS_ARBITRATION_LOST;
			}
		}
		//if status register does not contain valid DATA ACKNOWLEDGE
		if((TWSR&TWI_STATUS_BITS)!=SC_VALID_DATA_TX_AND_ACK_RX){
			//return error code
			return SC_VALID_DATA_TX_AND_ACK_RX;
		}//if
	}//for
	//----------------------------//
	//	CREATE 'STOP' CONDITION		//
	//----------------------------//
	//request for STOP transmission
	TWCR=(1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
	//return: transmission succeeded
	return 0;
}
