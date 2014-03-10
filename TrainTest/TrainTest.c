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
#include <stdbool.h>

#include "TrainTest.h"

#define DCC_PORT PORTA
#define DCC_PIN0 PORTA6
#define DCC_PIN1 PORTA7
#define DCC_DIRECTION DDRA

#define USE_DCC_TIMINGS

//volatile int64_t pwm;
//volatile uint8_t dirstate;

#define PACKET_BUFFER_SIZE (32)
#define MAX_DATA_BYTES (4)

#define PREAMBLE_LENGTH (16)

//the information required for a packet.  From this a whole real packet can be generated
typedef struct{
	uint8_t address;
	uint8_t dataBytes;//just the actual data bytes, we will work out the error detection bytes at transmission time
	uint8_t data[MAX_DATA_BYTES];
	//note that an error detection byte is a different type of data byte
}packetData_t;


//buffer to hold packets info to be sent
packetData_t packetBuffer[PACKET_BUFFER_SIZE];
//volatile int16_t packetBufferPosition;

//where in the packet are we currently?
enum transmitStates{
	PREAMBLE,
	PACKET_START_BIT,
	ADDRESS,
	DATA_START_BIT,
	DATA,
	ERROR_START_BIT,
	ERROR_DETECTION,//this and data will be interleaved if more than one data byte
	END_BIT
};

//which state are we currently in?
volatile uint8_t transmitState;

//the packet we are currently transmitting
volatile uint8_t transmittingPacket;
volatile uint8_t	packetsInBuffer;

//true when it's safe to insert a new packet into the packetBuffer
volatile bool safeToInsert;

//which bit of the preamble/address/data/errordetect are we transmitting?
volatile uint8_t transmittingBit;
//volatile uint8_t addressBit;
//which data byte are we transmitting?
volatile uint8_t transmittingDataByte;

volatile uint8_t debugMemory[128];
volatile uint16_t debugPosition=0;


//The different states which are cycled through when transmitting data
//a 1 is transmitted by being high during ONE_HIGH and then low during ONE_LOW
//each for 58us (the rate at which the interrupt is called)
//0 is tramsitted likewise, but high and low take twice as long
//(spec calls for >100us, so I'm using 116us because this is easy)
//See S91-2004-07 "A: Technique For Encoding Bits"
enum bitStates{
	ONE_HIGH,
	ONE_LOW,
	ZERO_HIGH1,
	ZERO_HIGH2,
	ZERO_LOW1,
	ZERO_LOW2
};

volatile uint8_t bitState;
//
////given position in packetBuffer, get a pointer to that packet
//packetData_t *getPacket(uint8_t packetId)
//{
	//return 
//}

void insertIdlePacket(uint8_t here)
{
	//packetBuffer[here].address=0xFF;
	//packetBuffer[here].data[0]=0x00;
	//packetBuffer[here].dataBytes=1;
	
	//go forwards command
	packetBuffer[here].address=3;
	packetBuffer[here].data[0]=0x7f;
	packetBuffer[here].dataBytes=1;
}

/*
 * Get a pointer to a position in the packet buffer where we can add a packet.  Also increment packetsInBuffer.
 */
packetData_t *getInsertPacketPointer()
{
	packetData_t* p = &(packetBuffer[(transmittingPacket+packetsInBuffer)%PACKET_BUFFER_SIZE]);
	packetsInBuffer++;
	return p;
}

//void insertPacket(packetData_t packet)
//{
	//packetBuffer[(transmittingPacket+packetsInBuffer)%PACKET_BUFFER_SIZE]=packet;
//}
//
int main(void)
{
		
	//Set timer0 to CTC mode (clears timer on compare match)
	//TCCR0A,WGM02;//=2;
	Clrb(TCCR0A,WGM00);
	Setb(TCCR0A,WGM01);
	Clrb(TCCR0A,WGM02);
	
	//OCIE0A: Timer/Counter0 Output Compare Match A Interrupt Enable
	Setb(TIMSK0,OCIE0A);
	#ifndef USE_DCC_TIMINGS
		//set counter's clock to be systemclock/1024
		Setb(TCCR0B,CS00);
		Clrb(TCCR0B,CS01);
		Setb(TCCR0B,CS02);
		//
		//what is the compare value for the timer?
		OCR0A=254;
	#else
		//set counter's clock to be systemclock/8 (so clock to timer will be 1MHz)
		Clrb(TCCR0B,CS00);
		Setb(TCCR0B,CS01);
		Clrb(TCCR0B,CS02);
	
		//what is the compare value for the timer?
		OCR0A=58;//58us, for half the period of a logical 1 for DCC
	#endif
	//this only gives 464 clock cycles between half periods - is this going to be enough?
	
	//enable interrupts globally
	sei();
	
	//set DCC pins to output
	Setb(DCC_DIRECTION,DCC_PIN0);
	Setb(DCC_DIRECTION,DCC_PIN1);

	
	Clrb(DCC_PORT,DCC_PIN0);
	
	//set up the packet buffer
	transmittingPacket=0;
	//put two idle packets in the buffer
	insertIdlePacket(transmittingPacket);
	insertIdlePacket(transmittingPacket+1);
	packetsInBuffer=2;
	
	uint8_t demoState=0;
	
	uint8_t i;
	packetData_t* nextPacket;
	
	while(1)
	{
		//if(packetsInBuffer < 3)
		//{
			////TODO work out how to deal with being interrupted at the wrong time
			//IDEA - have a flag which is raised at the start of transmitting a packet - then only insert while this is asserted
			//this will mean there are hundreds of clock cycles before a new idle packet will be automatically inserted
			//
		//}
		_delay_ms(2000);
		while(1);
		//wait for it to be safe to insert a new packet
		while(!safeToInsert);
		//now safe!
		demoState++;
		switch(demoState)
		{
			case 0:
				//go forwards!
				for(i=0;i<4;i++)
				{
					nextPacket=getInsertPacketPointer();
					nextPacket->address=3;
					//forwards at full speed
					//0111 1111
					nextPacket->data[0]=0x7F;
					nextPacket->dataBytes=1;
				}
				break;
			case 3:
				demoState=0;
			case 1:
				//stop
				for(i=0;i<4;i++)
				{
					nextPacket=getInsertPacketPointer();
					nextPacket->address=3;
					//0110 0000
					nextPacket->data[0]=0x60;
					nextPacket->dataBytes=1;
				}
				break;
			case 2:
				//go backwards!
				for(i=0;i<4;i++)
				{
					nextPacket=getInsertPacketPointer();
					nextPacket->address=3;
					//backwards at full speed
					//0101 1111
					nextPacket->data[0]=0x5F;
					nextPacket->dataBytes=1;
				}
				break;
		}
	}
}

inline packetData_t *currentPacket()
{
	return	&(packetBuffer[transmittingPacket]);
}
//we've reached the end of transmitting a bit, need to set up the state to transmit the next bit
uint8_t determineNextBit()
{
	safeToInsert=false;
	switch(transmitState)
	{
		case PREAMBLE:
			//only allow inserting new packets while transmitting preamble
			//in the hope that an interrupt will never try and add an idle packet at the same time as the main thread is adding new packets
			safeToInsert=true;
			transmittingBit++;
			if(transmittingBit > PREAMBLE_LENGTH)
			{
				//finished transmitting the preamble!
				transmitState = PACKET_START_BIT;
				transmittingBit=0;
				return ZERO_HIGH1;
			}else{
				//still mroe preamble bits to go
				return ONE_HIGH;
			}
			break;
		case PACKET_START_BIT:
			//finished transmitting the packet start bit, start transmitting address
			transmittingBit=0;
			transmitState = ADDRESS;
			//packetData_t *d = currentPacket();
			if(currentPacket()->address & (1 << 8))
			{
				//MSB of address is 1
				return ONE_HIGH;
			}
			else
			{
				return ZERO_HIGH1;
			}
			break;
		case ADDRESS:
			transmittingBit++;
			//packetData_t *d = currentPacket();
			if(transmittingBit >= 8){
				if(currentPacket()->dataBytes > 0)
				{
					
					//there is data to transmit
					transmitState=DATA_START_BIT;
					return ZERO_HIGH1;
				}else{
					//no data to transmit
					transmitState=END_BIT;
					return ONE_HIGH;
				}
				
				
			}else{
				//still address data to transmit
				if(currentPacket()->address & (1 << (8-transmittingBit)))
				{
					//jnext MSB of address is 1
					return ONE_HIGH;
				}
				else
				{
					return ZERO_HIGH1;
				}
			}
			break;
		case DATA_START_BIT:
			transmittingBit=0;
			transmittingDataByte=0;
			transmitState = DATA;
			if(currentPacket()->data[transmittingDataByte] & (1 << 8))
			{
				//MSB of data byte is 1
				return ONE_HIGH;
			}
			else
			{
				return ZERO_HIGH1;
			}
			break;
		case DATA:
			transmittingBit++;
			//TODO expand to allow multiple data bytes?
			if(transmittingBit >= 8){
				//reached end of this data byte
				transmitState = ERROR_START_BIT;
				return ZERO_HIGH1;
			}else{
				//still data bits to transmit
				if(currentPacket()->data[transmittingDataByte] & (1 << (8-transmittingBit)))
				{
					//jnext MSB of address is 1
					return ONE_HIGH;
				}
				else
				{
					return ZERO_HIGH1;
				}
			}
			break;
		case ERROR_START_BIT:
			transmittingBit=0;
			transmitState = ERROR_DETECTION;
			//error detection byte is xor of the data byte and address
			if((currentPacket()->data[transmittingDataByte] ^ currentPacket()->address) & (1 << 8))
			{
				//MSB of error detection byte is 1
				return ONE_HIGH;
			}
			else
			{
				return ZERO_HIGH1;
			}
			break;
		case ERROR_DETECTION:
			transmittingBit++;
		
			if(transmittingBit >= 8){
				//reached end of this error detection byte
				transmitState = END_BIT;
				return ONE_HIGH;
			}else{
				//still error detection bits to transmit
				if((currentPacket()->data[transmittingDataByte] ^ currentPacket()->address) & (1 << (8-transmittingBit)))
				{
					//jnext MSB of errordetect is 1
					return ONE_HIGH;
				}
				else
				{
					return ZERO_HIGH1;
				}
			}
			break;
		case END_BIT:
		default:
		
			packetsInBuffer--;
			transmitState=PREAMBLE;
			if(packetsInBuffer<=0)
			{
				//no more packets in buffer, add an idle packet
				//the main loop will handle adding other packets
				packetsInBuffer=1;
				insertIdlePacket(transmittingPacket);
			}else{
				transmittingPacket++;
				transmittingPacket%=PACKET_BUFFER_SIZE;
			}
			
			return ONE_HIGH;
			break;
	}
	
}

/************************************************************************/
/* Interrupt which is run every 58us                                    */
/************************************************************************/
ISR(TIMER0_COMPA_vect)
{

	//output the right bit
	switch(bitState)
	{
		case ONE_HIGH:
		case ZERO_HIGH1:
		case ZERO_HIGH2:

			Setb(DCC_PORT,DCC_PIN1);
			Clrb(DCC_PORT,DCC_PIN0);
			break;
		case ONE_LOW:
		case ZERO_LOW1:
		case ZERO_LOW2:
			Setb(DCC_PORT,DCC_PIN0);
			Clrb(DCC_PORT,DCC_PIN1);
		break;
	}
	//proceed to output the rest of this bit, or work out what the next bit is
	switch(bitState)
	{
		case ONE_HIGH:
			bitState = ONE_LOW;
			break;
		case ZERO_HIGH1:
			bitState=ZERO_HIGH2;
			break;
		case ZERO_HIGH2:
			bitState = ZERO_LOW1;
			break;
		case ZERO_LOW1:
			bitState = ZERO_LOW2;
			break;
			
		case ONE_LOW:
		case ZERO_LOW2:
		default:
			//need the next bit!
			bitState = determineNextBit();
			debugMemory[debugPosition/8]=bitState |= (bitState==ONE_HIGH ? 1 : 0) << (debugPosition%8);
			debugPosition++;
			if(debugPosition==128*8-1)
			{
				debugPosition=0;
			}
			break;
	}

}
