/*
* TrainTest.h
*
* Created: 01/03/2014 16:21:12
*  Author: Luke
*/


#ifndef TRAINTEST_H_
#define TRAINTEST_H_



#include "Include.h"





#define DCC_PORT PORTA
#define DCC_PIN0 PORTA6
#define DCC_PIN1 PORTA7
#define DCC_DIRECTION DDRA

#define USE_DCC_TIMINGS

#define PACKET_BUFFER_SIZE (32)
#define MAX_DATA_BYTES (4)

#define PREAMBLE_LENGTH (16)

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
    uint8_t dataBytes; //just the actual data bytes, we will work out the error detection bytes at transmission time
    uint8_t data[MAX_DATA_BYTES];
    //note that an error detection byte is a different type of data byte
} packetData_t;

#endif /* TRAINTEST_H_ */