/*
* TrainTest.h
*
* Created: 01/03/2014 16:21:12
*  Author: Luke
*/


#ifndef TRAINTEST_H_
#define TRAINTEST_H_



#include "Include.h"

#include "UART.h"


//port registers for the DCC pins
#define DCC_PORT PORTA
#define DCC_DIRECTION DDRA
#define DCC_PIN PINA
//service mode switch pin (input, active low)
#define DCC_nSERVICE_PIN PINA0
#define DCC_SERVICE_PULLUP PORTA0
//LEDs
#define DCC_DATA_LED PORTA1
#define DCC_SERVICE_MODE_LED PORTA2
#define DCC_IDLE_LED PORTA3
//DCC and nDCC pins
#define DCC_OUT_PIN PORTA6
#define DCC_nOUT_PIN PORTA7



#define USE_DCC_TIMINGS

#define PACKET_BUFFER_SIZE (128)
/*
While the baseline packet has a length of 3 data bytes separated by a "0" bit, a packet using the extended
packet format definition may have a length of between 3 and 6 data bytes each separated by a "0" bit.
 - RP-9.2.1 DCC Extended Packet Format
 (I think address counts as a packet)
*/
#define MAX_DATA_BYTES (5)

//minimum of 14 (though one can be last 1 of previous packet)
#define PREAMBLE_LENGTH (16)
//minimum of 20 bits
#define LONG_PREAMBLE_LENGTH (24)

//how many duplicate packets to transmit for the crude demo
#define DUPLICATION (8)

//if running DC test, how long between switching modes
#define DC_DELAY (1000)

//true when it's safe to insert a new packet into the packetBuffer
volatile bool safeToInsert;

void runDCCDemo(uint8_t address);
void simpleDCC_init();
bool setAddress(uint8_t newAddress);

/*
 * the information required for a packet.  From this a whole real packet can be generated
 */
typedef struct {
	bool longPreamble;
    uint8_t address;
    uint8_t dataBytes; //just the actual data bytes, we will work out the error detection byte at transmission time
    uint8_t data[MAX_DATA_BYTES];
    //note that an error detection byte is a different type of data byte
} dccPacket_t;

//not sure if this is oging to be needed - might simply pop into service mode and leave as soon as whatever action was completed
typedef enum baseStates {
    OPERATIONS_MODE,
	SERVICE_MODE,
	ENTER_SERVICE_MODE,
	LEAVE_SERVICE_MODE
} baseStates_t;

enum speedModes{
	SPEEDMODE_14STEP,
	SPEEDMODE_28STEP
	};

#endif /* TRAINTEST_H_ */