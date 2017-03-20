/*
 * TrainTest.c
 *
 * Created: 28/02/2014 18:17:37
 *  Author: Luke
 */




#include "SimpleDCC.h"
#include "UART.h"
#include "comms.h"

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

void adc_init(){
	/*
	If the result is left adjusted and no more than 8-bit precision is required, it is sufficient to read ADCH.
	
	Using the ADC Interrupt Flag as a trigger source makes the ADC start a new conversion as soon as the
	ongoing conversion has finished. The ADC then operates in Free Running mode, constantly sampling
	and updating the ADC Data Register
	
	If a lower resolution than 10 bits is needed, the input clock frequency
	to the ADC can be higher than 200kHz to get a higher sample rate.
	*/
	
	//reference voltage: AVcc with external capacitor at AREF pin
	Setb(ADMUX, REFS0);
	Clrb(ADMUX, REFS1);
	
	//left adjust result (we only want 8 MSBs anyway)
	Setb(ADMUX, ADLAR);
	
	//prescaler of 32 - clock of ADC of 250kHz (slightly above recommended, but we're not fussed about accuracy)
	
	#define SCALERMASK (0x7)//0111
	ADCSRA = (ADCSRA & ~SCALERMASK) | (0x5 & SCALERMASK);
	
	//set which single ended input to use
	//value = (value & ~mask) | (newvalue & mask);
	#define INPUTMASK  (0x1f) // 0001 1111
	//this is just setting the last 5 bits to the value of the adc input pin
	ADMUX = (ADMUX & ~INPUTMASK) | (CURRENT_SENSE_ADC_IN & INPUTMASK);
	
	//The Power Reduction ADC bit in the Power Reduction Register (PRR.PRADC) must be written to '0' in order to be enable the ADC.
	Clrb(PRR0, PRADC);
	
	//enable ADC
	Setb(ADCSRA, ADEN);
	
	
	
	//ADC will auto trigger from the source
	Setb(ADCSRA, ADATE);
	
	//adc should be in free running mode by default so the above should let it run forever
	
	//start it running
	Setb(ADCSRA, ADSC);
}


uint8_t adc_read(){
	return ADCH;
}

/*
* purely for testing, spit value of ADC out over UART human readable
*/
char adcbuffer[8];
void printADCValue(){
	
	sprintf(adcbuffer, "%d    \r", ADCH);
	for(int i=0;i<8;i++){
		USART_Transmit(adcbuffer[i]);
	}
	
}

int main(void) {
	
    simpleDCC_init();


#ifdef DC_TEST
    //if this is defined, just power the track with DC (used by me to test motors)
    DC_Test();
#endif

    timer_init();

//#define DCC_DEMO

    //set up UART
    USART_Init(BAUDRATE);
	//setAddress(8);
	
#define ADC_TEST

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
		
		processInput();

    }
}
