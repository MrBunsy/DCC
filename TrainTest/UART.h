/*
 * Setup.h
 *
 * Created: 22/03/2014 09:37:56
 *  Author: Luke
 */ 


#ifndef SETUP_H_
#define SETUP_H_


#include "Include.h"


void USART_Init(uint16_t baud);
uint8_t USART_Receive(void) ;
void USART_Transmit( uint8_t data );

#endif /* SETUP_H_ */