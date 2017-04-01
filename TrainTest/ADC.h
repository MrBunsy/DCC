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

volatile uint16_t currentDrawValue;

#endif /* ADC_H_ */