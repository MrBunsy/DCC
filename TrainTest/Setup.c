/*
 * CFile1.c
 *
 * Created: 22/03/2014 09:37:32
 *  Author: Luke
 */

#include "Setup.h"

void timer_init() {
    //Set timer0 to CTC mode (clears timer on compare match)
    Clrb(TCCR0A, WGM00);
    Setb(TCCR0A, WGM01);
    Clrb(TCCR0A, WGM02);

    //OCIE0A: Timer/Counter0 Output Compare Match A Interrupt Enable
    Setb(TIMSK0, OCIE0A);

    //set counter's clock to be systemclock/8 (so clock to timer will be 1MHz)
    Clrb(TCCR0B, CS00);
    Setb(TCCR0B, CS01);
    Clrb(TCCR0B, CS02);
	
    OCR0A = 58; //58us, for half the period of a logical 1 for DCC
    //this only gives 464 clock cycles between half periods
}

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
