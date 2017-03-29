/*
* TrainTest.c
*
* Created: 28/02/2014 18:17:37
*  Author: Luke
*/




#include "SimpleDCC.h"
#include "UART.h"
#include "comms.h"
#include "ADC.h"

//#define DCC_DEMO

#define BAUDRATE (19200)
//#define DC_TEST
//#define BAUDRATE (9600)
/*
* Enable the timer to call an interrupt every 58us
*/
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

	//enable interrupts globally
	sei();
}

//#define DC_TEST

int main(void) {
	
	Setb(LED_DIRECTION, LED_IDLE);
	Setb(LED_PORT, LED_IDLE);
	
//	while(1);

	#ifdef DC_TEST
		//if this is defined, just power the track with DC (used by me to test motors)
		DC_Test();
	#endif

	adc_init();
	simpleDCC_init();
	timer_init();
	
	

	//#define DCC_DEMO

	//set up UART
	USART_Init(BAUDRATE);
	//setAddress(8);
	
	//#define ADC_TEST

	#ifdef ADC_TEST
		adc_init();
	
		while(1){
			printADCValue();
		}
	#endif
	
	#ifdef DCC_DEMO
		//setAddress(5);
	
		runDCCDemo(6);
	#endif

	while (1) {

		//IDEA - have a flag which is raised at the start of transmitting a packet - then only insert while this is asserted
		//this will mean there are hundreds of clock cycles before a new idle packet will be automatically inserted
		
		processInput(false);

	}
}
