/*
 * SimpleDCC.c
 *
 * Created: 22/03/2014 11:46:43
 *  Author: Luke
 */

#include "SimpleDCC.h"




//buffer to hold packets info to be sent
packetData_t packetBuffer[PACKET_BUFFER_SIZE];

/*
 * where in the packet are we currently?
 */
enum transmitStates {
    PREAMBLE,
    PACKET_START_BIT,
    ADDRESS,
    DATA_START_BIT,
    DATA,
    ERROR_START_BIT,
    ERROR_DETECTION, //this and data will be interleaved if more than one data byte
    END_BIT
};

//which state are we currently in?
volatile uint8_t transmitState;

//the packet we are currently transmitting
volatile uint8_t transmittingPacket;
volatile uint8_t packetsInBuffer;

//which bit of the preamble/address/data/errordetect are we transmitting?
volatile uint8_t transmittingBit;
//volatile uint8_t addressBit;
//which data byte are we transmitting?
volatile uint8_t transmittingDataByte;

volatile uint8_t debugMemory[128];
volatile uint16_t debugPosition = 0;

/**
 * 
The different states which are cycled through when transmitting data
a 1 is transmitted by being high during ONE_HIGH and then low during ONE_LOW
each for 58us (the rate at which the interrupt is called)
0 is tramsitted likewise, but high and low take twice as long
(spec calls for >100us, so I'm using 116us because this is easy)
See S91-2004-07 "A: Technique For Encoding Bits"
 */
enum bitStates {
    ONE_HIGH,
    ONE_LOW,
    ZERO_HIGH1,
    ZERO_HIGH2,
    ZERO_LOW1,
    ZERO_LOW2
};

volatile uint8_t bitState;

void simpleDCC_init() {
    //set DCC pins to output
    Setb(DCC_DIRECTION, DCC_PIN0);
    Setb(DCC_DIRECTION, DCC_PIN1);

    //clear DCC output
    Clrb(DCC_PORT, DCC_PIN0);
    Clrb(DCC_PORT, DCC_PIN1);
}

/**
 * Given a position in the packet buffer, insert an idle packet
 */
void insertIdlePacket(uint8_t here) {
    packetBuffer[here].address = 0xFF;
    packetBuffer[here].data[0] = 0x00;
    packetBuffer[here].dataBytes = 1;
}

/*
 * Get a pointer to a position in the packet buffer where we can add a packet.  Also increment packetsInBuffer.
 */
packetData_t *getInsertPacketPointer() {
    packetData_t* p = &(packetBuffer[(transmittingPacket + packetsInBuffer) % PACKET_BUFFER_SIZE]);
    packetsInBuffer++;
    return p;
}

/*
 * Run in a loop to provide backwards and forwards commands to address 3
 */
void runDCCDemo() {


    int8_t demoState = 0;
    uint8_t i;
    packetData_t* nextPacket;

    while (1) {


        _delay_ms(1500);

        //wait for it to be safe to insert a new packet
        while (!safeToInsert);
        //now safe!
        (demoState)++;
        switch (demoState) {
            case 0:
                //go forwards!
                for (i = 0; i < DUPLICATION; i++) {
                    nextPacket = getInsertPacketPointer();
                    nextPacket->address = 3;
                    //forwards at full speed
                    //0111 1111
                    //nextPacket->data[0]=0x7F;
                    //half speed:
                    nextPacket->data[0] = 0x77;
                    nextPacket->dataBytes = 1;
                }

                break;
            case 3:
                demoState = -1;
            case 1:
                //stop
                for (i = 0; i < DUPLICATION; i++) {
                    nextPacket = getInsertPacketPointer();
                    nextPacket->address = 3;
                    //0110 0000
                    nextPacket->data[0] = 0x60;
                    nextPacket->dataBytes = 1;
                }

                break;
            case 2:
                //go backwards!
                for (i = 0; i < DUPLICATION; i++) {
                    nextPacket = getInsertPacketPointer();
                    nextPacket->address = 3;
                    nextPacket->data[0] = 0x57;
                    nextPacket->dataBytes = 1;
                }
                break;
        }
    }
}

/**
 * Power track in one direction for DC_DELAY, then wait and power in other direction.
 */
void DC_Test() {
    while (1) {
        _delay_ms(DC_DELAY);
        Clrb(DCC_PORT, DCC_PIN0);
        Setb(DCC_PORT, DCC_PIN1);

        _delay_ms(DC_DELAY);
        Setb(DCC_PORT, DCC_PIN0);
        Setb(DCC_PORT, DCC_PIN1);

        _delay_ms(DC_DELAY);
        Setb(DCC_PORT, DCC_PIN0);
        Clrb(DCC_PORT, DCC_PIN1);

        _delay_ms(DC_DELAY);
        Setb(DCC_PORT, DCC_PIN0);
        Setb(DCC_PORT, DCC_PIN1);
    }
}

/*
 * Return a pointer to the current packet being transmitted in the packet buffer
 */
inline packetData_t *currentPacket() {
    return &(packetBuffer[transmittingPacket]);
}

/**
 * we've reached the end of transmitting a bit, need to set up the state to transmit the next bit
 */
uint8_t determineNextBit() {
    safeToInsert = false;
    switch (transmitState) {
        case PREAMBLE:
            //only allow inserting new packets while transmitting preamble
            //in the hope that an interrupt will never try and add an idle packet at the same time as the main thread is adding new packets
            safeToInsert = true;
            transmittingBit++;
            if (transmittingBit > PREAMBLE_LENGTH) {
                //finished transmitting the preamble!
                transmitState = PACKET_START_BIT;
                transmittingBit = 0;
                return ZERO_HIGH1;
            } else {
                //still mroe preamble bits to go
                return ONE_HIGH;
            }
            break;
        case PACKET_START_BIT:
            //finished transmitting the packet start bit, start transmitting address
            transmittingBit = 0;
            transmitState = ADDRESS;
            //packetData_t *d = currentPacket();
            uint8_t address = currentPacket()->address;
            uint8_t bit = (1 << 7);

            if (address & bit) {
                //MSB of address is 1
                return ONE_HIGH;
            } else {
                return ZERO_HIGH1;
            }
            break;
        case ADDRESS:
            transmittingBit++;
            //packetData_t *d = currentPacket();
            if (transmittingBit >= 8) {
                if (currentPacket()->dataBytes > 0) {

                    //there is data to transmit
                    transmitState = DATA_START_BIT;
                    return ZERO_HIGH1;
                } else {
                    //no data to transmit
                    transmitState = END_BIT;
                    return ONE_HIGH;
                }


            } else {
                //still address data to transmit
                if (currentPacket()->address & (1 << (7 - transmittingBit))) {
                    //jnext MSB of address is 1
                    return ONE_HIGH;
                } else {
                    return ZERO_HIGH1;
                }
            }
            break;
        case DATA_START_BIT:
            transmittingBit = 0;
            transmittingDataByte = 0;
            transmitState = DATA;
            if (currentPacket()->data[transmittingDataByte] & (1 << 7)) {
                //MSB of data byte is 1
                return ONE_HIGH;
            } else {
                return ZERO_HIGH1;
            }
            break;
        case DATA:
            transmittingBit++;
            //TODO expand to allow multiple data bytes?
            if (transmittingBit >= 8) {
                //reached end of this data byte
                transmitState = ERROR_START_BIT;
                return ZERO_HIGH1;
            } else {
                //still data bits to transmit
                if (currentPacket()->data[transmittingDataByte] & (1 << (7 - transmittingBit))) {
                    //jnext MSB of address is 1
                    return ONE_HIGH;
                } else {
                    return ZERO_HIGH1;
                }
            }
            break;
        case ERROR_START_BIT:
            transmittingBit = 0;
            transmitState = ERROR_DETECTION;
            //error detection byte is xor of the data byte and address
            if ((currentPacket()->data[transmittingDataByte] ^ currentPacket()->address) & (1 << 7)) {
                //MSB of error detection byte is 1
                return ONE_HIGH;
            } else {
                return ZERO_HIGH1;
            }
            break;
        case ERROR_DETECTION:
            transmittingBit++;

            if (transmittingBit >= 8) {
                //reached end of this error detection byte
                transmitState = END_BIT;
                return ONE_HIGH;
            } else {
                //still error detection bits to transmit
                if ((currentPacket()->data[transmittingDataByte] ^ currentPacket()->address) & (1 << (7 - transmittingBit))) {
                    //jnext MSB of errordetect is 1
                    return ONE_HIGH;
                } else {
                    return ZERO_HIGH1;
                }
            }
            break;
        case END_BIT:
        default:

            packetsInBuffer--;
            transmitState = PREAMBLE;
            if (packetsInBuffer <= 0) {
                //no more packets in buffer, add an idle packet
                //the main loop will handle adding other packets
                packetsInBuffer = 1;
                insertIdlePacket(transmittingPacket);
            } else {
                transmittingPacket++;
                transmittingPacket %= PACKET_BUFFER_SIZE;
            }

            return ONE_HIGH;
            break;
    }

}

/************************************************************************/
/* Interrupt which is run every 58us                                    */

/************************************************************************/
ISR(TIMER0_COMPA_vect) {

    //output the right bit
    switch (bitState) {
        case ONE_HIGH:
        case ZERO_HIGH1:
        case ZERO_HIGH2:
            Setb(DCC_PORT, DCC_PIN1);
            Clrb(DCC_PORT, DCC_PIN0);
            break;
        case ONE_LOW:
        case ZERO_LOW1:
        case ZERO_LOW2:
            Setb(DCC_PORT, DCC_PIN0);
            Clrb(DCC_PORT, DCC_PIN1);
            break;
    }
    //proceed to output the rest of this bit, or work out what the next bit is
    switch (bitState) {
        case ONE_HIGH:
            bitState = ONE_LOW;
            break;
        case ZERO_HIGH1:
            bitState = ZERO_HIGH2;
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
            //debugMemory[debugPosition/8]=bitState |= (bitState==ONE_HIGH ? 1 : 0) << (debugPosition%8);
            debugMemory[debugPosition] = (bitState == ONE_HIGH ? 1 : 0);
            debugPosition++;
            if (debugPosition == 127) {
                debugPosition = 0;
            }
            break;
    }

}
