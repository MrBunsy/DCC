/*
 * Setup.h
 *
 * Created: 22/03/2014 09:37:56
 *  Author: Luke
 */ 


#ifndef SETUP_H_
#define SETUP_H_


#include "Include.h"
#include "SimpleDCC.h"

#define UART_TIMEOUT (200)

void USART_Init(uint32_t baud);
uint8_t USART_Receive(void) ;
void USART_Transmit( uint8_t data );
void USART_TransmitString(uint8_t* data);
bool USART_DataInAvailable(void);
uint8_t USART_ReceiveNonBlock(void);

#endif /* SETUP_H_ */