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

#define DCC_PORT PORTA
#define DCC_PIN0 PORTA6
#define DCC_PIN1 PORTA7
#define DCC_DIRECTION DDRA

#define USE_DCC_TIMINGS

volatile int64_t pwm;
volatile int8_t dirstate;

enum state{
	FORWARDS,
	STOP1,
	BACKWARDS,
	STOP2
	};

int main(void)
{
	
	pwm=0;
	dirstate = FORWARDS;
	
	//Set timer0 to CTC mode (clears timer on compare match)
	//TCCR0A,WGM02;//=2;
	Clrb(TCCR0A,WGM00);
	Setb(TCCR0A,WGM01);
	Clrb(TCCR0A,WGM02);
	
	//OCIE0A: Timer/Counter0 Output Compare Match A Interrupt Enable
	Setb(TIMSK0,OCIE0A);
	#ifndef USE_DCC_TIMINGS
	//this is set by hardware I think: (when an interrupt is called)
	//enable interrupt on compare match on timer0
	//Setb(TIFR0,OCF0A);
	
	//set counter's clock to be systemclock/1024
	Setb(TCCR0B,CS00);
	Clrb(TCCR0B,CS01);
	Setb(TCCR0B,CS02);
	
	//what is the compare value for the timer?
	OCR0A=254;
	#else
	//set counter's clock to be systemclock/8
	Clrb(TCCR0B,CS00);
	Setb(TCCR0B,CS01);
	Clrb(TCCR0B,CS02);
	
	//what is the compare value for the timer?
	OCR0A=58;//58us, for half the period of a logical 1 for DCC
	#endif
	//this only gives 464 clock cycles between half periods - is this going to be enough?
	
	//enable interrupts globally
	sei();
	
	////set led pin to output
	//setb(DCC_direction,DCC_pin);
	//
	//turn led pin on
	Setb(DCC_PORT,DCC_PIN0);
	Setb(DCC_PORT,DCC_PIN1);
	//
	//while(1)
	//{
	//_delay_ms(500);
	//clrb(DCC_port,DCC_pin);
	//_delay_ms(500);
	//setb(DCC_port,DCC_pin);
	//}
	
	Clrb(DCC_PORT,DCC_PIN0);
	
	while(1)
	{
		_delay_ms(500);
	}
}


ISR(TIMER0_COMPA_vect)
{
	pwm++;
	if(pwm %6000==0)
	{
		switch(dirstate)
		{
			
			case FORWARDS:
			Setb(DCC_PORT,DCC_PIN1);
			Clrb(DCC_PORT,DCC_PIN0);
			break;
			case BACKWARDS:
			Setb(DCC_PORT,DCC_PIN0);
			Clrb(DCC_PORT,DCC_PIN1);
			break;
			default:
			Clrb(DCC_PORT,DCC_PIN0);
			Clrb(DCC_PORT,DCC_PIN1);
			break;
		}
	
	//DCC_PORT ^= (1 << DCC_PIN1); // Toggle the LED
	dirstate++;
	dirstate%=4;
	}
}
