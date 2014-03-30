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

#define MAX_MESSAGE_DATA_BYTES (MAX_DATA_BYTES)

typedef enum {
    COMMAND_PROGRAMME_ADDRESS = 0, //go into service mode, send this new address, leave service mode
    COMMAND_SET_SPEED,
    COMMAND_ENABLE_LIGHTS,
    COMMAND_EMERGENCY_STOP,
    COMMAND_CUSTOM_PACKET,//arbitrarily defined packet, this way JMRI can deal with all the implementation, not me :D
} commandType_t;

#define SYNC_INT (0xffccccff)
#define NUM_SYNC_BYTES (4)

/*
 * Max size any message data can be
 */
typedef struct {
    uint8_t data[MAX_MESSAGE_DATA_BYTES]; //may need to change number of bytes
} genericMessageData_t;

typedef struct {
    uint8_t speed;
    bool forwards;
} speedMessageData_t;

typedef struct{
    bool on;
} lightsMessageData_t;

typedef struct {
    uint8_t newAddress;
} newAddressMessageData_t;

typedef union {
    genericMessageData_t genericMessageData;
    speedMessageData_t speedMessageData;
    lightsMessageData_t lightsMessageData;
    newAddressMessageData_t newAddressMessageData;
} messageDataUnion_t;

//#pragma pack(1)
typedef struct {
    //this will hold commandType_t
    uint8_t commandType;
    //dccPacket_t packet;//don't actually want both sides of the comms to have to implement full details of DCC
    uint8_t address;
    //usage TBD, intention being that braking packets will have priority over others, and similar
    int8_t priority;
    messageDataUnion_t data;
} message_t;



message_t readMessage(void);
void processInput();
#endif /* COMMS_H_ */