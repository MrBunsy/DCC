/*
 * comms.c
 *
 * Created: 22/03/2014 14:58:08
 *  Author: Luke
 */

#include "comms.h"

/*
 * Keep reading UART until we've finished a sync message
 */
void readUntilSync() {
    uint32_t syncBytes = SYNC_INT;

    uint8_t * syncPointer = (uint8_t*) & syncBytes;
    uint8_t syncPos = 0;
    uint8_t in;
    volatile uint8_t test;
    while (true) {
        in = USART_Receive();
        test = *(syncPointer + syncPos);
        if (in == test) {
            syncPos++;

            if (syncPos == NUM_SYNC_BYTES) {
                //read all the sync bytes
                return;
            }
        } else {
            syncPos = 0;
        }
    }
}

/**
 * Block waiting for a message over serial
 */
message_t readMessage(void) {
    uint8_t i;
    message_t message;

    uint8_t *messagePointer = (uint8_t *) & message;

    //volatile uint8_t debugBuffer[128];
    //
    //for(i=0;i<20;i++){
    //debugBuffer[i] = USART_Receive();
    //}

    //sync up!
    readUntilSync();

    for (i = 0; i<sizeof (message_t); i++) {
        uint8_t in = USART_Receive();
        *messagePointer = in;
        messagePointer++;
    }

    return message;
}

/*
 * Take control and just sit there processing input from uart
 */
void processInput() {
    while (true) {
        message_t message = readMessage();
        uint8_t i;
        USART_Transmit('r');
        switch (message.commandType) {
            case COMMAND_SET_SPEED:
                waitForSafeToInsert();
                for (i = 0; i < DUPLICATION; i++) {
                    insertSpeedPacket(message.address, message.data.speedMessageData.speed, message.data.speedMessageData.forwards, SPEEDMODE_128STEP);
                }
                break;
            case COMMAND_ENABLE_LIGHTS:
                waitForSafeToInsert();
                for (i = 0; i < DUPLICATION; i++) {
                    insertLightsPacket(message.address, message.data.lightsMessageData.on);
                }
                break;
                case COMMAND_EMERGENCY_STOP:
                //needs testing
                insertSpeedPacket(0, 1, false, SPEEDMODE_14STEP);
                break;
        }
    }
}