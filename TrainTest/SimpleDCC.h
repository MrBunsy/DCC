/*
 * TrainTest.h
 *
 * Created: 01/03/2014 16:21:12
 *  Author: Luke
 */


#ifndef TRAINTEST_H_
#define TRAINTEST_H_



#include "Include.h"
#include "uart.h"
#include "ADC.h"

#if (PROCESSOR == ATMEGA644) 
	//port registers for the DCC pins
	#define DCC_PORT PORTB
	#define DCC_DIRECTION DDRB
	#define DCC_PIN PINB
	
	#define DCC_MAIN_TRACK_OUT PORTB0
	#define DCC_MAIN_TRACK_ENABLE PORTB1
	
	#define DCC_PROG_TRACK_OUT PORTB2
	#define DCC_PROG_TRACK_ENABLE PORTB3
	
	#define LED_PORT PORTD
	#define LED_DIRECTION DDRD
	#define LED_PIN PIND
	
	//LEDs
	#define LED_DATA PORTD4 //yellow
	#define LED_SERVICE_MODE PORTD6 //blue
	#define LED_IDLE PORTD5 //green
	#define LED_OVERCURRENT PORTD3 //red

	//current sense input for ADC
	#define CURRENT_SENSE_MAIN_TRACK 0//ADC0_BIT
	#define CURRENT_SENSE_PROG_TRACK 1//ADC1_BIT
	
#elif (PROCESSOR == ATMEGA168)
/*	//TODO - REDO SUPPORT FOR THIS CHIP (if I ever want it?)

	//port registers for the DCC pins
	#define DCC_PORT PORTC
	#define DCC_DIRECTION DDRC
	#define DCC_PIN PINC
	//service mode switch pin (input, active low)
	#define DCC_nSERVICE_PIN PINC5
	#define DCC_SERVICE_PULLUP PORTC5
	//LEDs
	#define DCC_DATA_LED PORTC4
	#define DCC_SERVICE_MODE_LED PORTC3
	#define DCC_IDLE_LED PORTC2
	//DCC and nDCC pins
	#define DCC_OUT_PIN PORTC1
	#define DCC_nOUT_PIN PORTC0*/
#endif


//ADC reads 0-255, which maps to 0-Vcc. Assuming a 10ohm sense resistor, MAX_CURRENT should be 0.1*current_in_amps*255/Vcc
#define MAX_CURRENT (150) //amplifier on arduino shield ensures that max current is 3.3v, so we should probably use 3.3 as our reference
#define MAX_PROG_CURRENT (75) // 25 maps to 300mA, but this is below the usual noise on the main track, so going for half the main track current for now.


//buffer needs to be at least 50 to hold all the initialisation packets
//must be less than uint8 can hold!
#if (PROCESSOR == ATMEGA644) 
	#define PACKET_BUFFER_SIZE (128)
#elif (PROCESSOR == ATMEGA168)
//not enough RAM!
	#define PACKET_BUFFER_SIZE (100)
#endif
/*
While the baseline packet has a length of 3 data bytes separated by a "0" bit, a packet using the extended
packet format definition may have a length of between 3 and 6 data bytes each separated by a "0" bit.
 - RP-9.2.1 DCC Extended Packet Format
 (I think address counts as a packet)
 */
#define MAX_DATA_BYTES_IN_DCC_PACKET (5)

//minimum of 14 (though one can be last 1 of previous packet)
#define PREAMBLE_LENGTH (16)
//minimum of 20 bits
#define LONG_PREAMBLE_LENGTH (24)

//how many duplicate packets to transmit for the crude demo
#define DUPLICATION (8)

//if running DC test, how long between switching modes
#define DC_DELAY (1000)

#define  ACK_SAMPLE_THRESHOLD       (5)

//true when it's safe to insert a new packet into the packetBuffer
volatile bool safeToInsert;

//true if too much current has been drawn
volatile bool highCurrentDrawMainTrack;
volatile bool highCurrentDrawProgTrack;

extern volatile bool mainTrackPower;

/*
 * the information required for a packet.  From this a whole real packet can be generated
 */
typedef struct {
    bool longPreamble;
    uint8_t address;
    uint8_t dataBytes; //just the actual data bytes, we will work out the error detection byte at transmission time
    uint8_t data[MAX_DATA_BYTES_IN_DCC_PACKET];
    //note that an error detection byte is a different type of data byte
} dccPacket_t;

/************************************************************************/
/* general purpose stuct for returning read/write/verify results        */
/************************************************************************/
typedef struct {
	bool success;
	uint8_t cvValue;
	}cvResponse_t;

void runDCCDemo(uint8_t address);
void simpleDCC_init();
bool setAddress(uint8_t newAddress);
void insertSpeedPacket(uint8_t address, uint8_t speed, bool forwards, uint8_t mode);
void insertLightsPacket(uint8_t address, bool on);
void insertResetPacket(bool longPreamble);
dccPacket_t *getInsertPacketPointer();
uint8_t getPacketsInBuffer();
cvResponse_t setCVwithDirectMode(uint16_t cv, uint8_t newValue);
cvResponse_t setCVBitwithDirectMode(uint8_t bit, uint16_t cv, uint8_t newValue);
cvResponse_t readCVWithDirectMode(uint16_t cv);
void waitForSafeToInsert();
void emergencyCutPower();

bool isInServiceMode();
void leaveServiceMode();
bool enterServiceMode();

void setServiceLED();
void setDataLED();
void setIdleLED();

void setProgTrackPower(bool power);
void setMainTrackPower(bool power);

void intialiseDCC();

//not sure if this is oging to be needed - might simply pop into service mode and leave as soon as whatever action was completed
//service mode is going to need overhauling with dccpp I think

typedef enum baseStates {
    OPERATIONS_MODE,
    SERVICE_MODE,
    ENTER_SERVICE_MODE,
    LEAVE_SERVICE_MODE
} baseStates_t;

enum speedModes {
    SPEEDMODE_14STEP,
    SPEEDMODE_28STEP,
    SPEEDMODE_128STEP
};


#endif /* TRAINTEST_H_ */