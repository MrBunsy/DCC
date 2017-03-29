/*
 * Include.h
 *
 * Created: 22/03/2014 09:50:16
 *  Author: Luke
 */ 


#ifndef INCLUDE_H_
#define INCLUDE_H_

//define this before including delay
#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>
#include <stdbool.h>

#define Orb(port,bitnum)		port |= _BV(bitnum)
#define Setb(port,bitnum)		port |= _BV(bitnum)
#define Clrb(port,bitnum)		port &= ~(_BV(bitnum))
#define Rdb(pinp,bitnum)		(pinp & _BV(bitnum))
#define Invert(port,bitnum)		(port) ^= _BV(bitnum)

//#define bitRead Rdb

#define  ATMEGA168      0
#define  ATMEGA644		1

#define  PROCESSOR     ATMEGA644

#include "arduino/Arduino.h"

//things from DCCpp_Uno.h
#define COMM_TYPE 0
#define INTERFACE Serial


//stuff for using dccpp code
//#define byte uint8_t
//#ifndef max
	//#define max(a,b) ((a) > (b) ? (a) : (b))
//#endif
//#define SHOW_PACKETS  0
//#define highByte(b)	((uint8_t)(b >> 8))
//#define lowByte(b)	((uint8_t)(b  & 0x00ff))
//
//#define INTERFACE.print USART_TransmitString

//for debugging :P
#define DEBUG_LED_FLASH

#endif /* INCLUDE_H_ */