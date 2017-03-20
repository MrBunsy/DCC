/*
 * TrainTest.h
 *
 * Created: 01/03/2014 16:21:12
 *  Author: Luke
 */


#ifndef TRAINTEST_H_
#define TRAINTEST_H_



#include "Include.h"

#include "UART.h"

#if (PROCESSOR == ATMEGA644) 
	//port registers for the DCC pins
	#define DCC_PORT PORTA
	#define DCC_DIRECTION DDRA
	#define DCC_PIN PINA
	//service mode switch pin (input, active low)
	#define DCC_nSERVICE_PIN PINA0
	#define DCC_SERVICE_PULLUP PORTA0
	//LEDs
	#define DCC_DATA_LED PORTA1
	#define DCC_SERVICE_MODE_LED PORTA2
	#define DCC_IDLE_LED PORTA3
	//DCC and nDCC pins
	#define DCC_OUT_PIN PORTA6
	#define DCC_nOUT_PIN PORTA7
	//current sense input for ADC
	#define CURRENT_SENSE_ADC_IN ADC5_BIT
#elif (PROCESSOR == ATMEGA168)
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
	#define DCC_nOUT_PIN PORTC0
#endif


#ifdef SECOND_DATA_LED
	//little hack to support a second LED on a new board before the small coloured LEDs arrive
	#define DEBUG_LED_PORT	PORTB
	#define DEBUG_LED	PORTB1
	#define DEBUG_LED_DIR	DDRB
#endif

#define USE_DCC_TIMINGS
//buffer needs to be at least 50 to hold all the initialisation packets
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
#define MAX_DATA_BYTES (5)

//minimum of 14 (though one can be last 1 of previous packet)
#define PREAMBLE_LENGTH (16)
//minimum of 20 bits
#define LONG_PREAMBLE_LENGTH (24)

//how many duplicate packets to transmit for the crude demo
#define DUPLICATION (8)

//if running DC test, how long between switching modes
#define DC_DELAY (1000)

//if defined, service mode will always be available, regardless of if the service_mode_pin is pulled low
#define OVERRIDE_SERVICE_MODE_PIN

//true when it's safe to insert a new packet into the packetBuffer
volatile bool safeToInsert;

/*
 * the information required for a packet.  From this a whole real packet can be generated
 */
typedef struct {
    bool longPreamble;
    uint8_t address;
    uint8_t dataBytes; //just the actual data bytes, we will work out the error detection byte at transmission time
    uint8_t data[MAX_DATA_BYTES];
    //note that an error detection byte is a different type of data byte
} dccPacket_t;

void runDCCDemo(uint8_t address);
void simpleDCC_init();
bool setAddress(uint8_t newAddress);
void insertSpeedPacket(uint8_t address, uint8_t speed, bool forwards, uint8_t mode);
void insertLightsPacket(uint8_t address, bool on);
void insertResetPacket(bool longPreamble);
dccPacket_t *getInsertPacketPointer();
bool setCVwithDirectMode(uint16_t cv, uint8_t newValue);
void waitForSafeToInsert();

bool isInServiceMode();
void leaveServiceMode();
bool enterServiceMode();

void setServiceLED();
void setDataLED();
void setIdleLED();

//not sure if this is oging to be needed - might simply pop into service mode and leave as soon as whatever action was completed

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

//enum functions {
    //LIGHT_FRONT = 0,
    //LIGHT_REAR,
	//FUNCTION_3,
	//FUNCTION_4
//};

#endif /* TRAINTEST_H_ */