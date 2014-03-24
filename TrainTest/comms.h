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

typedef enum {
    DATA_RUNNING,// usual data for running trains
    PROGRAMME_ADDRESS //go into service mode, send this new address, leave service mode
} commandType_t;

typedef struct {
    //this will hold commandType_t
    uint8_t commandType;
    dccPacket_t packet;
    //usage TBD, intention being that braking packets will have priority over others, and similar
    int8_t priority;
} commsPacket_t;



void receivePackets(void);
#endif /* COMMS_H_ */