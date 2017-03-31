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


//uint16_t debug;

void USART_Init(uint32_t baud) {

    uint16_t ubrr = (uint16_t) ((uint32_t) (F_CPU / (16 * baud) -1));
#if (PROCESSOR == ATMEGA644) 
    //enable clock to UART0
    Clrb(PRR0, PRUSART0);
#endif
    /* Set baud rate */
    UBRR0H = (uint8_t) (ubrr >> 8);
    UBRR0L = (uint8_t) ubrr;
    /* Enable receiver and transmitter */
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    /* Set frame format: 8data, 1stop bit */
    //UCSR0C = (3 << UCSZ00);

    UCSR0C = (0 << UMSEL00) // asynchronous 
            | (0 << UMSEL01) // asynchronous     
            | (0 << UPM01) // parity bit disabled
            | (0 << UPM00) // parity bit disabled
            | (0 << USBS0) // 1 stop bit
            | (1 << UCSZ01) // 8 bits
            | (1 << UCSZ00) // 8 bits
            | (0 << UCSZ02); //8 bits
    //UCSR0A  U2X0
	//debug=0;
}

uint8_t USART_Receive(void) {
    /* Wait for data to be received */
    while (!(UCSR0A & (1 << RXC0)))
        ;
    /* Get and return received data from buffer */
    return UDR0;
}

/************************************************************************/
/* Return true if data is available to be read from Rx                  */
/************************************************************************/
bool USART_DataInAvailable(void){
	return (UCSR0A & (1 << RXC0));
}

/************************************************************************/
/* Read data from Rx, to be used with USART_DataInAvailable             */
/************************************************************************/
uint8_t USART_ReceiveNonBlock(void) {
	/* Get and return received data from buffer */
	return UDR0;
}

void USART_Transmit(uint8_t data) {
    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1 << UDRE0)))
        ;
    /* Put data into buffer, sends the data */
    UDR0 = data;
	//debug++;
	//if(debug==256)
	//{
		//debug=0;
	//}
}

void USART_TransmitString(uint8_t* data){
	//TODO
}
