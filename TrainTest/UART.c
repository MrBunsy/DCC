/*
 * CFile1.c
 *
 * Created: 22/03/2014 09:37:32
 *  Author: Luke
 */

#include "UART.h"

/* Initialize UART (from datasheet and altered)
 * 
 * @param baudrate From equation UBBR = f_osc/(16BAUD) - 1
 */
void USART_Init(uint16_t baud) {
    //enable clock to UART0
    Clrb(PRR0, PRUSART0);
    /* Set baud rate */
    UBRR0H = (uint8_t) (baud >> 8);
    UBRR0L = (uint8_t) baud;
    /* Enable receiver and transmitter */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    /* Set frame format: 8data, 1stop bit */
    UCSR0C = (3 << UCSZ00);
}

uint8_t USART_Receive(void) {
    /* Wait for data to be received */
    while (!(UCSR0A & (1 << RXC0)))
        ;
    /* Get and return received data from buffer */
    return UDR0;
}

void USART_Transmit( uint8_t data )
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) )
	;
	/* Put data into buffer, sends the data */
	UDR0 = data;
}
