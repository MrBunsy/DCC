/*
 * ADC.h
 *
 * Created: 20/03/2017 19:29:56
 *  Author: Luke
 */ 


#ifndef ADC_H_
#define ADC_H_

#include "Include.h"
#include "SimpleDCC.h"

void adc_init();
uint8_t adc_read();
void printADCValue();
//uint8_t getProgTrackValue();
uint8_t getAvgProgTrackCurrent(void);

volatile uint16_t currentDrawValue;


#define PROG_TRACK_CURRENT_BUFFER_SIZE (32)

volatile extern uint8_t mainTrackCurrent;
volatile extern uint8_t progTrackCurrent;
//volatile extern uint8_t progTrackCurrentBuffer[PROG_TRACK_CURRENT_BUFFER_SIZE];
//volatile extern bool progTrackCurrentUpdated;


#define ADC_PORT PORTA
#define ADC_PORT_DIR DDRA

//set which single ended input to use
//value = (value & ~mask) | (newvalue & mask);
// 0001 1111
//this is just setting the last 5 bits to the value of the adc input pin
#define ADC_INPUT_MASK  (0x1f)

#define adc_input_set(setto)	ADMUX = (ADMUX & ~ADC_INPUT_MASK) | (setto & ADC_INPUT_MASK);


#endif /* ADC_H_ */