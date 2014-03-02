/*
 * TrainTest.c
 *
 * Created: 28/02/2014 18:17:37
 *  Author: Luke
 */ 

//define this before including delay
#define F_CPU 8000000UL

#include <avr/io.h>
#include <util/delay.h>
#include <stdint.h>
#include <avr/interrupt.h>

#include "TrainTest.h"

#define LED_PORT PORTA
#define LED_PIN PORTA7
#define LED_DIRECTION DDRA

int main(void)
{
	
	//Set timer0 to CTC mode (clears timer on compare match)
	//TCCR0A,WGM02;//=2;
	Clrb(TCCR0A,WGM00);
	Setb(TCCR0A,WGM01);
	Clrb(TCCR0A,WGM02);
	
	//OCIE0A: Timer/Counter0 Output Compare Match A Interrupt Enable
	Setb(TIMSK0,OCIE0A);
	
	//this is set by hardware I think: (when an interrupt is called)
	////enable interrupt on compare match on timer0
	//Setb(TIFR0,OCF0A);
	
	//set counter's clock to be systemclock/1024
	Setb(TCCR0B,CS00);
	Clrb(TCCR0B,CS01);
	Setb(TCCR0B,CS02);
	
	//what is the compare value for the timer?
	OCR0A=200;
	
	//enable interrupts globally
	sei(); 
	
	////set led pin to output
	//setb(led_direction,led_pin);
	//
	////turn led pin on
	//setb(led_port,led_pin);
	//
    //while(1)
    //{
        //_delay_ms(500);
		//clrb(led_port,led_pin);
		//_delay_ms(500);
		//setb(led_port,led_pin);
    //}
	
	while(1)
	{
		_delay_ms(500);
	}
}


ISR(TIMER0_COMPA_vect)
{
	LED_PORT ^= (1 << LED_PIN); // Toggle the LED
}
