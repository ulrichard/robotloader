//----------------------------------------------------------------------
//	INCLUDE FILES
//----------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include "RobotArmBase.h"

//------------------------------------------------------------
//	FUNCTION PROTOTYPES
//------------------------------------------------------------
unsigned char ucShowPositiveDecimalValueInDisplay(unsigned int uiDecVal);
void vInitDisplay(void);


//-------------------------------------------------------------------------------//
//	TWI (I2C) INTERFACE DEFINES
//-------------------------------------------------------------------------------//
//SC = Status Code, these are official I2C values
#define SC_VALID_SINGLE_START_TX               0x08
#define SC_VALID_REPEATED_START_TX             0x10
#define SC_VALID_SLAVE_ADDRESS_TX_AND_ACK_RX   0x18
#define SC_VALID_SLAVE_ADDRESS_TX_AND_NACK_RX  0x20
#define SC_VALID_DATA_TX_AND_ACK_RX            0x28
#define SC_VALID_DATA_TX_AND_NACK_RX           0x30
#define SC_BUS_ARBITRATION_LOST                0x38

#define TWI_STATUS_BITS                        0xf8

#define TIMER2_OC2_OUTPUT_USED_BY_BEEPER       0x01
#define TIMER2_OC2_OUTPUT_USED_BY_RS232        0x02
#define TIMER2_OC2_OUTPUT_USED_BY_ULTRASOON    0x04

