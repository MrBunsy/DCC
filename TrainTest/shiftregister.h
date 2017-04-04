/*
 * shiftregister.h
 * 
 * This is for controlling the signals and point motors, which are backed by a shift-register LED driver
 *
 * Created: 04/04/2017 13:45:26
 *  Author: Luke
 */ 


#ifndef SHIFTREGISTER_H_
#define SHIFTREGISTER_H_

#include "Include.h"
#include "comms.h"
#include "spi.h"

#define MAX_SHIFT_REGISTER_BYTES (50)

#define SHIFT_REGISTER_ENABLE PORTB4
#define SHIFT_REGISTER_DIR DDRB
#define SHIFT_REGISTER_PORT PORTB

//don't think this is needed anywhere else
//extern uint8_t shiftregisterData[MAX_SHIFT_REGISTER_BYTES];

extern uint16_t shiftRegisterSize;

void resetShiftRegister(uint16_t newSize);
void setShiftRegisterData(uint16_t dataPosition, uint8_t* data);
void outputShiftRegister(void);
void shiftRegister_init(void);




#endif /* SHIFTREGISTER_H_ */