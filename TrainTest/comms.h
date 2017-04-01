/*
* comms.h
*
* Created: 22/03/2014 14:58:15
*  Author: Luke
*/


#ifndef COMMS_H_
#define COMMS_H_

#include "Include.h"
#include "UART.h"
#include "SimpleDCC.h"
#include <string.h>

//message data is only part of the message, this is not the size of the full message (which is data + type + sync bytes)
#define MAX_MESSAGE_DATA_BYTES (MAX_DATA_BYTES_IN_DCC_PACKET + 3)

typedef enum {
	COMMAND_PROGRAMME_DIRECT_BYTE = 0, //go into service mode, send this new address, leave service mode
	COMMAND_OPERATIONS_MODE_PACKET, //arbitrarily defined packet, this way JMRI can deal with all the implementation, not me :D
	COMMAND_PROGRAMME_ADDRESS,
	COMMAND_SET_SPEED,
	COMMAND_ENABLE_LIGHTS,
	COMMAND_EMERGENCY_STOP,
	COMMAND_ENTER_SERVICE_MODE,
	
	RESPONSE_PACKET_BUFFER_SIZE, //inform the listener how many packets are currently in the buffer
	RESPONSE_COMMS_ERROR,
} commandType_t;

//used for finding the start of a packet:
#define SYNC_INT (0xffccccff)
#define NUM_SYNC_BYTES (4)

//sync bytes + data + command type
#define FULL_MESSAGE_LENGTH (MAX_MESSAGE_DATA_BYTES + NUM_SYNC_BYTES + 1)

#define INPUT_BUFFER_SIZE (30)
/*
* Max size any message data can be
*/
typedef struct {
	uint8_t data[MAX_MESSAGE_DATA_BYTES]; //may need to change number of bytes
} genericMessageData_t;

typedef struct {
	uint8_t address;
	uint8_t speed;
	bool forwards;
} speedMessageData_t;

typedef struct {
	uint8_t address;
	bool on;
} lightsMessageData_t;

typedef struct {
	uint8_t newAddress;
} newAddressMessageData_t;

typedef struct {
	//jmri also sends the error correction byte - I could use this too? (would simplify my code a huge amount if JMRI did *all* the DCC encoding...)
	uint8_t address;
	//having data preceeded by address means that a COMMAND_CUSTOM_PACKET doesn't need to care that this system treats address seperate to the data
	//in that a custom packet can be sent with commandType of COMMAND_CUSTOM_PACKET, then priority then the full data (where the first byte is usually address anyway)
	uint8_t data[MAX_DATA_BYTES_IN_DCC_PACKET];
	uint8_t dataBytes;
	uint8_t repeat;
} opsModePacketMessageData_t;

typedef struct{
	uint16_t cv;
	uint8_t newValue;
} programmeDirectByteMessageData_t;

typedef struct{
	uint8_t packetsInBuffer;
}packetBufferSize_t;


typedef union {
	genericMessageData_t genericMessageData;
	speedMessageData_t speedMessageData;
	lightsMessageData_t lightsMessageData;
	newAddressMessageData_t newAddressMessageData;
	opsModePacketMessageData_t opsModePacketMessageData;
	programmeDirectByteMessageData_t programmeDirectByteMessageData;
	packetBufferSize_t packetBufferSizeData;
} messageDataUnion_t;

//#pragma pack(1)

typedef struct {
	//this will hold commandType_t
	uint8_t commandType;
	messageDataUnion_t data;
} message_t;


void transmitMessage(uint8_t* messagePointer);
void transmitPacketBufferSize(uint8_t size);
message_t readMessage(void);
void processInput(void);
void transmitCommsDebug(void);
void bufferInput(void);
void processMessage(message_t* message);

#endif /* COMMS_H_ */