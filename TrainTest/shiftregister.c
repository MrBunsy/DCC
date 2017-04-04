/*
* shiftregister.c
*
* Created: 04/04/2017 13:46:22
*  Author: Luke
*/

#include "shiftregister.h"
//TLC5917: Clock input for data shift on rising edge

uint8_t shiftregisterData[MAX_SHIFT_REGISTER_BYTES];
uint16_t shiftRegisterSize = MAX_SHIFT_REGISTER_BYTES;

void shiftRegister_init(void){
	//set pins to output
	Setb(SHIFT_REGISTER_DIR, SHIFT_REGISTER_ENABLE);
	
	resetShiftRegister(MAX_SHIFT_REGISTER_BYTES);
}

/************************************************************************/
/* process the data from an incomming message.                          */
/* data position is where in the array is being updated		            */
/* data is of length SHIFT_REG_BYTES_PER_MESSAGE						*/
/* NOTE - this makes no effort to check all the messages have been		*/
/* received, there may be some missing.									*/
/************************************************************************/
void setShiftRegisterData(uint16_t dataPosition, uint8_t* data){
	uint8_t* insertHere = &shiftregisterData[dataPosition];
	
	//check we won't try to run off the end
	uint8_t relevantBytes = SHIFT_REG_BYTES_PER_MESSAGE;
	if(dataPosition + SHIFT_REG_BYTES_PER_MESSAGE > shiftRegisterSize){
		relevantBytes = shiftRegisterSize - dataPosition;
	}
	
	for(uint8_t i=0;i<relevantBytes;i++){
		insertHere[i] = data[i];
	}
}

/************************************************************************/
/* set the new size and set all contents to zero                        */
/************************************************************************/
void resetShiftRegister(uint16_t newSize){
	if(newSize > MAX_SHIFT_REGISTER_BYTES){
		newSize = MAX_SHIFT_REGISTER_BYTES;
	}
	for(uint16_t i =0;i<newSize;i++){
		shiftregisterData[i] = 0;
	}
	shiftRegisterSize=newSize;
}

/************************************************************************/
/* actually send the data out over SPI                                  */
/************************************************************************/
void outputShiftRegister(void){
	SPI_MasterTransmitArray(shiftregisterData,shiftRegisterSize);
	
	//toggle shift register enable pin
	Setb(SHIFT_REGISTER_PORT, SHIFT_REGISTER_ENABLE);
	_delay_us(1); 
	Clrb(SHIFT_REGISTER_PORT, SHIFT_REGISTER_ENABLE);
}