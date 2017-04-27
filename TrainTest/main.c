/*
* TrainTest.c
*
* Created: 28/02/2014 18:17:37
*  Author: Luke
*/



#include "Include.h"
#include "SimpleDCC.h"
#include "uart.h"
#include "comms.h"
#include "ADC.h"
#include "shiftregister.h"
#include "spi.h"

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
	
	Setb(LED_DIRECTION, LED_DATA);
	Setb(LED_PORT, LED_DATA);
	
	//	while(1);

	#ifdef DC_TEST
	//if this is defined, just power the track with DC (used by me to test motors)
	DC_Test();
	#endif

	adc_init();
	simpleDCC_init();
	timer_init();
	shiftRegister_init();
	spi_init();
	
	
	
	#ifdef SHIFT_TEST
	
	uint8_t data = 0x0f;
	
	resetShiftRegister(1);
	setShiftRegisterData(0,&data);
	outputShiftRegister();
	#endif
	

	//#define DCC_DEMO

	//set up UART
	//USART_Init(BAUDRATE);
	uart_init( UART_BAUD_SELECT(BAUDRATE, F_CPU) );
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
	
	transmitStatus(getPacketsInBuffer(),mainTrackCurrent, mainTrackPower);
	transmitStatus(getPacketsInBuffer(),mainTrackCurrent, mainTrackPower);
	transmitStatus(getPacketsInBuffer(),mainTrackCurrent, mainTrackPower);
	
	#define BLOCKING_COMMS
	
	while (1) {
		
		
		#ifdef BLOCKING_COMMS
		message_t message = readMessage();
		processMessage(&message);
		#else
		//not got this working and don't understand why, yet
		bufferInput();
		processInput();
		#endif
		//I'd like to do current checking here, but while processInput could potentially block it's best left in the DCC 'thread'
		//I think processInput will potentially only block if garbage is on the serial port, so if a proper message is sent at startup, that might clear it?
		
		//inform the listener if the packet buffer is getting low
		uint8_t packetsInBuffer = getPacketsInBuffer();
		//if (packetsInBuffer <= 2){
		transmitStatus(packetsInBuffer,mainTrackCurrent, mainTrackPower);
		//something goes seriously wrong with the comms if I try sending another message here.
		//unsure what caused it or if it even was on the AVR.
		//transmitCurrentDraw(currentDrawValue);
		//}
	}
}
