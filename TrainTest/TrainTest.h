/*
* TrainTest.h
*
* Created: 01/03/2014 16:21:12
*  Author: Luke
*/


#ifndef TRAINTEST_H_
#define TRAINTEST_H_



#include "Include.h"

//this equates to nearly 19200
#define BAUDRATE 25

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

#endif /* TRAINTEST_H_ */