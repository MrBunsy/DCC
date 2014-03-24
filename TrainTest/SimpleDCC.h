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
//DCC and nDCC pins
#define DCC_OUT_PIN PORTA6
#define DCC_nOUT_PIN PORTA7
//service mode switch pin
#define SERVICE_PIN PINA0
#define SERVICE_PULLUP PORTA0


#define USE_DCC_TIMINGS

#define PACKET_BUFFER_SIZE (64)
#define MAX_DATA_BYTES (4)

#define PREAMBLE_LENGTH (16)
#define LONG_PREAMBLE_LENGTH (32)

//how many duplicate packets to transmit
#define DUPLICATION (8)

//if running DC test, how long between switching modes
#define DC_DELAY (1000)

//true when it's safe to insert a new packet into the packetBuffer
volatile bool safeToInsert;

void runDCCDemo();
void simpleDCC_init();

/*
 * the information required for a packet.  From this a whole real packet can be generated
 */
typedef struct {
    uint8_t address;
    uint8_t dataBytes; //just the actual data bytes, we will work out the error detection byte at transmission time
    uint8_t data[MAX_DATA_BYTES];
    //note that an error detection byte is a different type of data byte
} dccPacket_t;

typedef enum baseStates {
    RUNNING_MODE,
	SERVICE_MODE,
	ENTER_SERVICE_MODE,
	LEAVE_SERVICE_MODE
} baseStates_t;

#endif /* TRAINTEST_H_ */