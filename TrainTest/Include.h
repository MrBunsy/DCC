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

#define  ATMEGA168      0
#define  ATMEGA644		1

#define  PROCESSOR     ATMEGA644



//for debugging :P
#define DEBUG_LED_FLASH

#endif /* INCLUDE_H_ */