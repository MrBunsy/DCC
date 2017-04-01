/*
* TrainTest.c
*
* Created: 28/02/2014 18:17:37
*  Author: Luke
*/




#include "SimpleDCC.h"
#include "uart.h"
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
	
	
	#ifdef UART_ECHO
	uint16_t c;
	for(;;)
    {
        /*
         * Get received character from ringbuffer
         * uart_getc() returns in the lower byte the received character and 
         * in the higher byte (bitmask) the last receive error
         * UART_NO_DATA is returned when no data is available.
         *
         */
        c = uart_getc();
        if ( c & UART_NO_DATA )
        {
            /* 
             * no data available from UART 
             */
        }
        else
        {
            /*
             * new data available from UART
             * check for Frame or Overrun error
             */
            if ( c & UART_FRAME_ERROR )
            {
                /* Framing Error detected, i.e no stop bit detected */
                uart_puts_P("UART Frame Error: ");
            }
            if ( c & UART_OVERRUN_ERROR )
            {
                /* 
                 * Overrun, a character already present in the UART UDR register was 
                 * not read by the interrupt handler before the next character arrived,
                 * one or more received characters have been dropped
                 */
                uart_puts_P("UART Overrun Error: ");
            }
            if ( c & UART_BUFFER_OVERFLOW )
            {
                /* 
                 * We are not reading the receive buffer fast enough,
                 * one or more received character have been dropped 
                 */
                uart_puts_P("Buffer overflow error: ");
            }
            /* 
             * send received character back
             */
			uart_putc( (unsigned char)'\r' );
			uart_putc( (unsigned char)'\n' );
            uart_putc( (unsigned char)c );
        }
    }
	#endif
	
	
	
	while (1) {
		
		//bufferInput();
		//processInput();
		message_t message = readMessage();
		processMessage(&message);
		//I'd like to do current checking here, but while processInput could potentially block it's best left in the DCC 'thread'
		//I think processInput will potentially only block if garbage is on the serial port, so if a proper message is sent at startup, that might clear it?
		
		//inform the listener if the packet buffer is getting low
		uint8_t packetsInBuffer = getPacketsInBuffer();
		//if (packetsInBuffer <= 2){
		transmitPacketBufferSize(packetsInBuffer);
		//}
	}
}
