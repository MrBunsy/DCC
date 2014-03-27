/*
 * SimpleDCC.c
 *
 * Created: 22/03/2014 11:46:43
 *  Author: Luke
 */

#include "SimpleDCC.h"

//at the packet level, what is happening? EG running, entering service mode (baseStates_t)
volatile uint8_t operatingState;

//buffer to hold packets info to be sent
dccPacket_t packetBuffer[PACKET_BUFFER_SIZE];

/*
 * where in the packet are we currently?
 */
enum dataTransmitStates {
    PREAMBLE,
    PACKET_START_BIT,
    ADDRESS,
    DATA_START_BIT,
    DATA,
    ERROR_START_BIT,
    ERROR_DETECTION, //this and data will be interleaved if more than one data byte
    END_BIT
};

//which state are we currently in (which part of the packet are we transmitting)?
volatile uint8_t transmitPacketState;

//the packet we are currently transmitting (as a position in the packetbuffer)
volatile uint8_t transmittingPacket;
volatile uint8_t packetsInBuffer;

//which bit of the preamble/address/data/errordetect are we transmitting?
volatile uint8_t transmittedBits;
//which data byte are we transmitting?
volatile uint8_t transmittingDataByte;
//build up the error detection byte as we transmit
volatile uint8_t errorDetectionByte;

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

/*
 * Get a pointer to a position in the packet buffer where we can add a packet.  Also increment packetsInBuffer.
 */
dccPacket_t *getInsertPacketPointer() {
    while (packetsInBuffer == PACKET_BUFFER_SIZE);
    dccPacket_t* p = &(packetBuffer[(transmittingPacket + packetsInBuffer) % PACKET_BUFFER_SIZE]);
    packetsInBuffer++;
    return p;
}

/**
 * Insert an idle packet at the end of the packetbuffer
 */
void insertIdlePacket(bool longPreamble) {
    dccPacket_t *packet = getInsertPacketPointer();
    packet->address = 0xFF;
    packet->data[0] = 0x00;
    packet->dataBytes = 1;
    packet->longPreamble = longPreamble;
}

/*
 * Insert a reset packet at the end of the packetbuffer
 */
void insertResetPacket(bool longPreamble) {
    dccPacket_t *packet = getInsertPacketPointer();
    packet->address = 0x00;
    packet->data[0] = 0x00;
    packet->dataBytes = 1;
    packet->longPreamble = longPreamble;
}

void simpleDCC_init() {
    //set DCC pins to output
    Setb(DCC_DIRECTION, DCC_OUT_PIN);
    Setb(DCC_DIRECTION, DCC_nOUT_PIN);
    Setb(DCC_DIRECTION, DCC_IDLE_LED);
    Setb(DCC_DIRECTION, DCC_SERVICE_MODE_LED);
    Setb(DCC_DIRECTION, DCC_DATA_LED);

    //set service mode switch pin to input
    Clrb(DCC_DIRECTION, DCC_nSERVICE_PIN);

    //enable pullup for service mode switch (making it active low, and if the hardware isn't present, we be unable to enter service mode)
    Setb(DCC_PORT, DCC_SERVICE_PULLUP);

    //clear DCC output
    Clrb(DCC_PORT, DCC_OUT_PIN);
    Clrb(DCC_PORT, DCC_nOUT_PIN);

    /*
        In the case where there is no information about the previous state of the system, the Digital Command
        Station shall send a minimum of twenty (20) digital decoder reset packets to the layout followed by a
        minimum of ten (10) idle packets.
         - RP-9.2.4 DCC Fail Safe
     */

    operatingState = OPERATIONS_MODE;
    transmittingPacket = 0;
    packetsInBuffer = 0;
    int i;
    for (i = 0; i < 20; i++) {
        insertResetPacket(false);
    }

    for (i = 0; i < 10; i++) {
        insertIdlePacket(false);
    }

    //ready to go straight into service mode - NOT REQUIRED
    for (i = 0; i < 20; i++) {
        insertIdlePacket(false);
    }
}

/*
 * Returns true if we can enter service mode (the switch is toggled)
 */
bool canEnterServiceMode() {
    //is the service mode pin low?
    return (Rdb(DCC_PIN, DCC_nSERVICE_PIN) == 0);
}

/*
 * direct mode service mode to set the address CV
 *
 * returns false if unsuccessful (although atm this will only happen if we can't enter service mode due to mech switch)
 */
bool setCVwithDirectMode(uint16_t cv, uint8_t newValue) {
    //don't allow this unless we can enter service mode
    if (!canEnterServiceMode()) {
        return false;
    }

    uint8_t i;
    dccPacket_t *packet;


    //at least three reset packets with long preamble
    for (i = 0; i < 5; i++) {
        insertResetPacket(true);
    }

    for (i = 0; i < 15; i++) {
        packet = getInsertPacketPointer();
        /*long-preamble 0 0111CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
        two bit address (AA) in the first data byte being the most significant bits of the CV number.
        CC=10 Bit Manipulation
        CC=01 Verify byte
        CC=11 Write byte
         */
        //01111100 = 7c
        packet->address = 0b01111100 | ((cv >> 8) & 0b11); //write to CV, with the 2 MSB of CV number
        packet->data[0] = cv & 0xff; //lowest 8 bits of cv
        packet->data[1] = newValue;
        packet->dataBytes = 2;
        packet->longPreamble = true;
    }

    //at least 6 reset packets
    //    for (i = 0; i < 8; i++) {
    //        insertResetPacket(true);
    //    }
    operatingState = SERVICE_MODE;
    return true;
}

bool setAddress(uint8_t newAddress) {
    return setCVwithDirectMode(0, newAddress);
}

/*
 * Turn on or off front light (this is all atm, need to understand a bit more before implementing other features)
 */
void insertLightsPacket(uint8_t address, bool on) {
	/*
	send a Function Group One Instruction
	The format of this instruction is 100DDDDD
	see RP-9.2.1 Extended Packet Format
	*/
	dccPacket_t *nextPacket = getInsertPacketPointer();
	nextPacket->address = address;
	nextPacket->data[0] = 0b10000000;
	
	if(on){
		//the FL bit
		//note: "If Bit 1 of CV#29 has a value of one (1), then bit 4 controls function FL, otherwise bit 4 has no meaning."
		nextPacket->data[0] |= 0b10010000;
	}	
	nextPacket->dataBytes=1;
	nextPacket->longPreamble=false;
}

/*
 * Speed is 3-28 (in 14step mode the LSB is ignored) in 14 adn 28 speedmodes
 Speed is 2-127 in 128 speedmode
 speed 0 is stop
 speed 1,2 is emergency stop
 */
void insertSpeedPacket(uint8_t address, uint8_t speed, bool forwards, uint8_t mode) {
    dccPacket_t *nextPacket = getInsertPacketPointer();
    nextPacket->address = address;

    if (mode == SPEEDMODE_128STEP) {
        //send an Advanced Operations Instruction (001)
        //The format of this instruction is 001CCCCC 0 DDDDDDDD
        //see RP-9.2.1 Extended Packet Format
        //CCCCC = 11111: 128 Speed Step Control
        nextPacket->data[0] = 0b00111111;
        //7 LSBs are speed
        nextPacket->data[1] = speed & 0b01111111;
        //bit7 is direction
        nextPacket->data[1] |= (forwards ? 0x1 << 7 : 0);
        nextPacket->dataBytes = 2;
    } else {
        //01DCSSSS
        //C is an aditional lowest significant bit used in 28speed mode.  in 14speed mode it is the light
        nextPacket->data[0] = 0b01000000;
        if (forwards) {
            nextPacket->data[0] |= 0b00100000;
        }

        //remove LSB and stick speed in lower nibble
        nextPacket->data[0] |= (speed >> 1) & 0x0f;

        //Speeds range from 2-16 (in 14speed step mode) and 2-28 in 28mode
        switch (mode) {
            case SPEEDMODE_14STEP:
                //set C (LSB of speed) to be the direction to control the light
                if (forwards) {
                    nextPacket->data[0] |= 0b00010000;
                }

                break;
            case SPEEDMODE_28STEP:
                //stick LSB of speed in C
                nextPacket->data[0] |= (speed & 0x01) << 4;
                break;
        }
        nextPacket->dataBytes = 1;
    }
    nextPacket->longPreamble = false;
}

/*
 * wait for it to be safe to insert a new packet
 */
void waitForSafeToInsert(){
	
	while (!safeToInsert);
}

/*
 * Run in a loop to provide backwards and forwards commands to address 3
 */
void runDCCDemo(uint8_t address) {


    int8_t demoState = 0;
    uint8_t i;
    //dccPacket_t* nextPacket;



    //setAddress(4);
    //while(1);
    //return;

    while (1) {

		
        _delay_ms(1500);
		insertLightsPacket(address,true);
		
        //wait for it to be safe to insert a new packet
        while (!safeToInsert);
        //now safe!
        (demoState)++;
        switch (demoState) {
            case 0:
                //go forwards!
                USART_Transmit('f');
                for (i = 0; i < DUPLICATION; i++) {
                    //insertSpeedPacket(address, 15, true, SPEEDMODE_14STEP);
                    insertSpeedPacket(address, 100, true, SPEEDMODE_128STEP);
                }
                break;
            case 3:
                USART_Transmit('\n');
                demoState = -1;
            case 1:
                //stop
                USART_Transmit('s');
                for (i = 0; i < DUPLICATION; i++) {
                    insertSpeedPacket(address, 0, true, SPEEDMODE_128STEP);
                }
                break;
            case 2:
                //go backwards!
                USART_Transmit('b');
                for (i = 0; i < DUPLICATION; i++) {
                    insertSpeedPacket(address, 100, false, SPEEDMODE_128STEP);
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
        Clrb(DCC_PORT, DCC_OUT_PIN);
        Setb(DCC_PORT, DCC_nOUT_PIN);

        _delay_ms(DC_DELAY);
        Setb(DCC_PORT, DCC_OUT_PIN);
        Setb(DCC_PORT, DCC_nOUT_PIN);

        _delay_ms(DC_DELAY);
        Setb(DCC_PORT, DCC_OUT_PIN);
        Clrb(DCC_PORT, DCC_nOUT_PIN);

        _delay_ms(DC_DELAY);
        Setb(DCC_PORT, DCC_OUT_PIN);
        Setb(DCC_PORT, DCC_nOUT_PIN);
    }
}

/*
 * Return a pointer to the current packet being transmitted in the packet buffer
 */
//inline 
dccPacket_t *getCurrentPacket() {
    return &(packetBuffer[transmittingPacket]);
}

void setDataLED() {
    Setb(DCC_PORT, DCC_DATA_LED);
    Clrb(DCC_PORT, DCC_SERVICE_MODE_LED);
    Clrb(DCC_PORT, DCC_IDLE_LED);
}

void setServiceLED() {
    Setb(DCC_PORT, DCC_SERVICE_MODE_LED);
    Clrb(DCC_PORT, DCC_DATA_LED);
    Clrb(DCC_PORT, DCC_IDLE_LED);
}

void setIdleLED() {
    Setb(DCC_PORT, DCC_IDLE_LED);
    Clrb(DCC_PORT, DCC_SERVICE_MODE_LED);
    Clrb(DCC_PORT, DCC_DATA_LED);
}

/*
 * The packet buffer has just run out, fill it with something depending on what state we're in
 */
void fillPacketBuffer() {
    switch (operatingState) {
        case OPERATIONS_MODE:
            //normal running mode, we just want to keep sending out idle packets
            //TODO here is where more inteligent logic about which commands to prioritise can go
            insertIdlePacket(false);
            setIdleLED();
            break;
        case SERVICE_MODE:
            //clear DCC output
            Clrb(DCC_PORT, DCC_OUT_PIN);
            Clrb(DCC_PORT, DCC_nOUT_PIN);
            //leave it turned off for half a second (spec says optional power off, and this didn't seem to work before doing this)
            //we're *in* the interrupt routine, so this should work fine - this is also why I can't turn off interrupts from here
            _delay_ms(500);
            operatingState = OPERATIONS_MODE;
            insertIdlePacket(false);
            break;
    }
}

/**
 * we've reached the end of transmitting a bit, need to set up the state to transmit the next bit
 
 NOTE transmittedBits is the bits that *will* have been transmitted after this function has returned
 
 
 TODO - THE ADDRESS CAN/SHOULD JUST BE PART OF THE DATA, then it's just preamble + various data bytes + error detection
 
 the error detection byte *appears* (this remains untested) to be all the data + Address xored togehter, furthering the
  idea that address shouldn't be a special case
 
 
 thought - the spec does differentiate between data and address, might it be easier to comprehend if I also do?
 */
uint8_t determineNextBit() {
    safeToInsert = false;
    dccPacket_t *currentPacket = getCurrentPacket();
    switch (transmitPacketState) {
        case PREAMBLE:
            //only allow inserting new packets while transmitting preamble
            //in the hope that an interrupt will never try and add an idle packet at the same time as the main thread is adding new packets
            safeToInsert = true;
            transmittedBits++;
            //long preamble is for service mode packets
            if (transmittedBits >= (currentPacket->longPreamble ? LONG_PREAMBLE_LENGTH : PREAMBLE_LENGTH)) {
                //this is the last bit of the preamble
                transmitPacketState = PACKET_START_BIT;
            }
            return ONE_HIGH;
            break;
        case PACKET_START_BIT:
            //transmitting the packet start bit, the next state will be the address
            transmittedBits = 0;
            transmitPacketState = ADDRESS;
            return ZERO_HIGH1;
            break;
        case ADDRESS:
            transmittedBits++;

            if (transmittedBits >= 8) {
                //this is the last bit of the address, which state next?
                if (currentPacket->dataBytes > 0) {
                    //there is data to transmit
                    //start building up the error detection byte
                    errorDetectionByte = currentPacket->address;
                    transmittingDataByte = 0;
                    transmitPacketState = DATA_START_BIT;
                } else {
                    //no data to transmit
                    transmitPacketState = END_BIT;
                }
            }

            //address data to transmit
            if (currentPacket->address & (1 << (8 - transmittedBits))) {
                //next MSB of address is 1
                return ONE_HIGH;
            } else {
                return ZERO_HIGH1;
            }

            break;
        case DATA_START_BIT:
            transmittedBits = 0;
            //this will always be 1 based, since it gets set to zero at the end of address
            transmittingDataByte++;
            transmitPacketState = DATA;
            return ZERO_HIGH1;
            break;
        case DATA:
            transmittedBits++;

            if (transmittedBits >= 8) {
                //reached end of this data byte
                //xor with error detection byte as per spec
                //I'm fairly certain that every data byte (& address byte) should be xored together to make the final error detection byte
                errorDetectionByte ^= currentPacket->data[transmittingDataByte - 1];
                if (currentPacket->dataBytes > transmittingDataByte) {
                    //more data bytes to transmit
                    transmitPacketState = DATA_START_BIT;
                } else {
                    //no more to transmit, start the error correction byte
                    transmitPacketState = ERROR_START_BIT;
                }
            }
            //still data bits to transmit
            if (currentPacket->data[transmittingDataByte - 1] & (1 << (8 - transmittedBits))) {
                //jnext MSB of address is 1
                return ONE_HIGH;
            } else {
                return ZERO_HIGH1;
            }

            break;
        case ERROR_START_BIT:
            transmittedBits = 0;
            //next transmit the error detection byte
            transmitPacketState = ERROR_DETECTION;

            return ZERO_HIGH1;
            break;
        case ERROR_DETECTION:
            transmittedBits++;

            if (transmittedBits >= 8) {
                //reached end of this error detection byte
                transmitPacketState = END_BIT;

            }
            //still error detection bits to transmit
            if (errorDetectionByte & (1 << (8 - transmittedBits))) {
                //next MSB of error detect is 1
                return ONE_HIGH;
            } else {
                return ZERO_HIGH1;
            }

            break;
        case END_BIT:
            //the end bit can form part of the next preamble
        default:
            transmittedBits = 0;
            packetsInBuffer--;
            transmitPacketState = PREAMBLE;
            if (packetsInBuffer <= 0) {
                //no more packets in buffer, add an idle packet
                //the main loop will handle adding other packets
                //packetsInBuffer = 1;
                //insertIdlePacket(transmittingPacket);

                //find what packet to put in next (usualy an idle packet)
                fillPacketBuffer();
            } else {
                transmittingPacket++;
                transmittingPacket %= PACKET_BUFFER_SIZE;

                //detect what the next packet type is and set LED accordingly
                if (packetBuffer[transmittingPacket].longPreamble) {
                    setServiceLED();
                } else if (packetBuffer[transmittingPacket].address == 0xff) {
                    //idle packet
                    setIdleLED();
                } else {
                    //assume everything thing else is data
                    setDataLED();

                }
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
            Setb(DCC_PORT, DCC_nOUT_PIN);
            Clrb(DCC_PORT, DCC_OUT_PIN);
            break;
        case ONE_LOW:
        case ZERO_LOW1:
        case ZERO_LOW2:
            Setb(DCC_PORT, DCC_OUT_PIN);
            Clrb(DCC_PORT, DCC_nOUT_PIN);
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
