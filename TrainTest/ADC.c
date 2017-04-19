/*
* ADC.c
*
* Created: 20/03/2017 19:33:04
*  Author: Luke
*/

#include "ADC.h"
#include "stdio.h"

//true for main track, false for prog track
volatile bool readingMainTrack = true;
volatile uint8_t mainTrackCurrent = 0;
volatile uint8_t progTrackCurrent = 0;
volatile bool progTrackCurrentUpdated = false;
volatile uint8_t progTrackCurrentBufferPos = 0;
volatile uint8_t progTrackCurrentBuffer[PROG_TRACK_CURRENT_BUFFER_SIZE];

void adc_init(){
	/*
	If the result is left adjusted and no more than 8-bit precision is required, it is sufficient to read ADCH.
	
	Using the ADC Interrupt Flag as a trigger source makes the ADC start a new conversion as soon as the
	ongoing conversion has finished. The ADC then operates in Free Running mode, constantly sampling
	and updating the ADC Data Register
	
	If a lower resolution than 10 bits is needed, the input clock frequency
	to the ADC can be higher than 200kHz to get a higher sample rate.
	*/
	
	//ensure definitely input with no pullups
	ADC_PORT_DIR = 0x00;
	ADC_PORT = 0x00;
	
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
	// 0001 1111
	//this is just setting the last 5 bits to the value of the adc input pin
	//ADMUX = (ADMUX & ~ADC_INPUT_MASK) | (CURRENT_SENSE_MAIN_TRACK & ADC_INPUT_MASK);
	adc_input_set(CURRENT_SENSE_MAIN_TRACK);
	readingMainTrack = true;
	progTrackCurrentUpdated = false;
	
	//The Power Reduction ADC bit in the Power Reduction Register (PRR.PRADC) must be written to '0' in order to be enable the ADC.
	Clrb(PRR0, PRADC);
	
	//enable ADC
	Setb(ADCSRA, ADEN);
	
	//enable interrupt
	Setb(ADCSRA, ADIE);
	
	
	
	//ADC will auto trigger from the source
	//Setb(ADCSRA, ADATE);
	
	//adc should be in free running mode by default so the above should let it run forever
	
	//start a conversion (or free-running forever if ADATE is set in ADCSR)
	Setb(ADCSRA, ADSC);
}


uint8_t adc_read(){
	return ADCH;
}

/************************************************************************/
/* block until a new prog track current read is in, then return it      */
/************************************************************************/
/*uint8_t getProgTrackValue(){
	while(!progTrackCurrentUpdated);
	progTrackCurrentUpdated = false;
	return progTrackCurrent;
}*/

uint8_t getAvgProgTrackCurrent(void){
	uint16_t baseCurrent = 0;
	for(uint8_t j = 0;j<PROG_TRACK_CURRENT_BUFFER_SIZE;j++){
		baseCurrent+=progTrackCurrentBuffer[j];
	}
	baseCurrent/=PROG_TRACK_CURRENT_BUFFER_SIZE;
	
	return (uint8_t)baseCurrent;
}

ISR(ADC_vect)
{
	if(readingMainTrack){
		mainTrackCurrent = ADCH;
		adc_input_set(CURRENT_SENSE_PROG_TRACK);
		readingMainTrack = false;
		}else{
		progTrackCurrent = ADCH;
		adc_input_set(CURRENT_SENSE_MAIN_TRACK);
		//for any other 'thread' waiting for a new value
		//progTrackCurrentUpdated = true;
		progTrackCurrentBuffer[progTrackCurrentBufferPos]=progTrackCurrent;
		progTrackCurrentBufferPos++;
		progTrackCurrentBufferPos%=PROG_TRACK_CURRENT_BUFFER_SIZE;
		readingMainTrack = true;
	}
	
	//start next conversion
	Setb(ADCSRA, ADSC);
}