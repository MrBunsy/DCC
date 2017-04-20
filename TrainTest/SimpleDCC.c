/*
* SimpleDCC.c
*
* Created: 22/03/2014 11:46:43
*  Author: Luke
*/

#include "SimpleDCC.h"



//buffer to hold packets info to be sent
dccPacket_t mainTrackPacketBuffer[PACKET_BUFFER_SIZE];
dccPacket_t programmingPacketBuffer[PACKET_BUFFER_SIZE/2];

dccTransmitionState_t mainTrackState;
dccTransmitionState_t progTrackState;




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
	ERROR_DETECTION,
	END_BIT
};



#ifdef DEBUG_MEMORY
volatile uint8_t debugMemory[128];
volatile uint16_t debugPosition = 0;
#endif
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



/*
* Get a pointer to a position in the packet buffer where we can add a packet.  Also increment packetsInBuffer.
* blocks if packet buffer is full
*/
dccPacket_t *getInsertPacketPointer(dccTransmitionState_t* state) {
	//taking out the wait, as I think in general it's better to overwrite stuff in the buffer than block when not expected
	//while (state->packetsInBuffer == PACKET_BUFFER_SIZE);
	dccPacket_t* p = &(state->packetBuffer[(state->transmittingPacket + state->packetsInBuffer) % PACKET_BUFFER_SIZE]);
	state->packetsInBuffer++;
	return p;
}

/************************************************************************/
/* same as getInsertPacketPointer but garenteed to not block            */
/************************************************************************/
dccPacket_t *getInsertPacketPointerNoBlock(dccTransmitionState_t* state){
	dccPacket_t* p = &(state->packetBuffer[(state->transmittingPacket + state->packetsInBuffer) % PACKET_BUFFER_SIZE]);
	state->packetsInBuffer++;
	return p;
}

/************************************************************************/
/* returns how many messages are in the buffer                          */
/************************************************************************/
uint8_t getPacketsInBuffer(dccTransmitionState_t* state){
	return state->packetsInBuffer;
}

/**
* Insert an idle packet at the end of the packetbuffer
*/
void insertIdlePacket(dccTransmitionState_t* state, bool longPreamble) {
	dccPacket_t *packet = getInsertPacketPointer(state);
	packet->address = 0xFF;
	packet->data[0] = 0x00;
	packet->dataBytes = 1;
	packet->longPreamble = longPreamble;
}

/*
* Insert a reset packet at the end of the packetbuffer
*/
void insertResetPacket(dccTransmitionState_t* state, bool longPreamble) {
	dccPacket_t *packet = getInsertPacketPointer(state);
	packet->address = 0x00;
	packet->data[0] = 0x00;
	packet->dataBytes = 1;
	packet->longPreamble = longPreamble;
}

/*
* Insert a page preset packet at the end of the packetbuffer
*/
void insertPagePresetPacket(dccTransmitionState_t* state, bool longPreamble) {
	dccPacket_t *packet = getInsertPacketPointer(state);
	packet->address = 0b01111101;
	packet->data[0] = 0b00000001;
	packet->dataBytes = 1;
	packet->longPreamble = longPreamble;
}

/*
* Run commands required as per spec to start operations mode
*/
void enterOperationsMode(dccTransmitionState_t* state){
	
	Setb(DCC_PORT, DCC_MAIN_TRACK_ENABLE);// in case was turned off at the end of programming mode
	
	state->operatingState = OPERATIONS_MODE;
	state->transmittingPacket = 0;
	state->packetsInBuffer = 0;
	
	int i;
	for (i = 0; i < 20; i++) {
		insertResetPacket(state, false);
	}

	for (i = 0; i < 10; i++) {
		insertIdlePacket(state, false);
	}

	//ready to go straight into service mode - NOT REQUIRED
	for (i = 0; i < 20; i++) {
		insertIdlePacket(state, false);
	}
}

/************************************************************************/
/* After powering on a track, perform the setup                         */
/************************************************************************/
void intialiseDCC(dccTransmitionState_t* state){
	/*
	In the case where there is no information about the previous state of the system, the Digital Command
	Station shall send a minimum of twenty (20) digital decoder reset packets to the layout followed by a
	minimum of ten (10) idle packets.
	- RP-9.2.4 DCC Fail Safe
	*/
	int i;
	for (i = 0; i < 20; i++) {
		insertResetPacket(&mainTrackState, false);
	}

	for (i = 0; i < 10; i++) {
		insertIdlePacket(&mainTrackState, false);
	}

	//ready to go straight into service mode - NOT REQUIRED
	for (i = 0; i < 20; i++) {
		insertIdlePacket(&mainTrackState, false);
	}
}

void simpleDCC_init() {
	//set DCC pins to output
	Setb(DCC_DIRECTION, DCC_MAIN_TRACK_OUT);
	Setb(DCC_DIRECTION, DCC_MAIN_TRACK_ENABLE);
	Setb(DCC_DIRECTION, DCC_PROG_TRACK_OUT);
	Setb(DCC_DIRECTION, DCC_PROG_TRACK_ENABLE);
	Setb(LED_DIRECTION, LED_IDLE);
	Setb(LED_DIRECTION, LED_SERVICE_MODE);
	Setb(LED_DIRECTION, LED_DATA);
	Setb(LED_DIRECTION, LED_OVERCURRENT);


	//start DCC output
	Clrb(DCC_PORT, DCC_MAIN_TRACK_OUT);
	Clrb(DCC_PORT, DCC_MAIN_TRACK_ENABLE);
	
	Clrb(DCC_PORT, DCC_PROG_TRACK_OUT);
	Clrb(DCC_PORT, DCC_PROG_TRACK_ENABLE);
	

	
	mainTrackState.packetBuffer = mainTrackPacketBuffer;
	mainTrackState.outputPin = DCC_MAIN_TRACK_OUT;
	mainTrackState.operatingState = OPERATIONS_MODE;
	mainTrackState.transmittingPacket = 0;
	mainTrackState.packetsInBuffer = 0;
	mainTrackState.serviceModeOnly = false;
	
	progTrackState.packetBuffer = programmingPacketBuffer;
	progTrackState.outputPin = DCC_PROG_TRACK_OUT;
	progTrackState.operatingState = OFF;
	progTrackState.transmittingPacket = 0;
	progTrackState.packetsInBuffer = 0;
	progTrackState.serviceModeOnly=true;
	
	intialiseDCC(&mainTrackState);
	
	//setCVwithDirectMode(1,6);
}

/************************************************************************/
/* turn power on or off for main track and prog track                   */
/************************************************************************/
void setProgTrackPower(bool power){
	if(power){
		Setb(DCC_PORT, DCC_PROG_TRACK_ENABLE);
		Clrb(LED_PORT, LED_OVERCURRENT);
		}else{
		Clrb(DCC_PORT, DCC_PROG_TRACK_ENABLE);
	}
}
void setMainTrackPower(bool power){
	if(power){
		//TODO intialiseDCC() cleanly
		Setb(DCC_PORT, DCC_MAIN_TRACK_ENABLE);
		Clrb(LED_PORT, LED_OVERCURRENT);
		}else{
		Clrb(DCC_PORT, DCC_MAIN_TRACK_ENABLE);
	}
}

//some globals used when reading/writing to decoders
serviceModeInfo_t serviceModeInfo;

/************************************************************************/
/* turn the programming track on and insert start-service-mode packets  */
/************************************************************************/
void enterServiceMode(dccTransmitionState_t * state){
	
	
	//to stop more than one service mode action occuring at once (if we received a message to start somethign else before this has finished)
	serviceModeInfo.inUse=true;
	uint8_t i;
	waitForSafeToInsert(state);
	//turn the track on
	setProgTrackPower(true);
	
	//Upon applying power to the track, the Command Station/Programmer must transmit at least 20
	//valid packets to the Digital Decoder to allow it time to stabilize internal operation before any Service Mode
	//operations are initiated.
	//using the usual initialise
	intialiseDCC(state);
	
	//at least three reset packets with long preamble
	for (i = 0; i <5; i++) {
		insertResetPacket(state,true);
	}
	state->operatingState=SERVICE_MODE;
}
/*
void leaveServiceMode(){
operatingState=OPERATIONS_MODE;
}

bool isInServiceMode(){
return operatingState==SERVICE_MODE;
}*/

/************************************************************************/
/* take lots of samples of the current and average them. Blocking       */
/************************************************************************/
/*uint8_t getAvgCurrent(){
uint16_t j;
uint16_t baseCurrent = 0;
for(j = 0;j<PROG_TRACK_CURRENT_BUFFER_SIZE;j++){
baseCurrent+=getProgTrackValue();
}
baseCurrent/=ACK_BASE_COUNT;

return (uint8_t)baseCurrent;
}*/

/************************************************************************/
/* Send the packets to trigger a response from the decoder for reading a single bit in direct mode  */
/************************************************************************/
void insertReadBitUsingDirectModePackets(dccTransmitionState_t* state, uint16_t cv, uint8_t bit){
	uint8_t j;
	for(j =0;j<3;j++){
		insertResetPacket(state,true);
	}
	dccPacket_t *packet;
	for (j = 0; j < 8; j++) {
		packet = getInsertPacketPointer(state);
		packet->address = 0x78 | ((cv >> 8) & 0b11); //read CV, with the 2 MSB of CV number
		packet->data[0] = cv & 0xff; //lowest 8 bits of cv
		packet->data[1] = 0xE8+bit;//which bit to read
		packet->dataBytes = 2;
		packet->longPreamble = true;
	}
	
	for(j=0;j<1;j++){
		insertResetPacket(state,true);
	}
}

void readCVWithDirectMode(dccTransmitionState_t* state, uint16_t cv, uint16_t callback, uint16_t callbacksub){
	

	//cvReadResponse_t response;
	
	//response.cvValue = 0;
	/*state->operatingState=OPERATIONS_MODE;
	setProgTrackPower(true);
	intialiseDCC(state);
	for(uint8_t k=0;k<50;k++){
	insertLightsPacket(state,6,true);
	}
	return response;*/
	
	cv--;
	
	//dccPacket_t *packet;
	//uint8_t i,j;
	//uint8_t cvValue = 0;
	//uint8_t baseCurrents[8];
	//volatile uint8_t currents[8];
	
	serviceModeInfo.CVBeingProcessed=cv;
	serviceModeInfo.bitBeingProcessed=0;
	serviceModeInfo.cvValue=0;
	
	serviceModeInfo.callback=callback;
	serviceModeInfo.callbacksub=callbacksub;
	
	enterServiceMode(state);
	serviceModeInfo.state=READ_BYTE_SETUP;
	/*

	//uint8_t baseCurrent = getAvgProgTrackCurrent();
	
	for(i=0;i<8;i++){
	
	while(getPacketsInBuffer(state) > 10);
	
	uint8_t current = getAvgProgTrackCurrent();
	
	if(current > baseCurrent && current - baseCurrent > ACK_SAMPLE_THRESHOLD){
	//Setb(response.cv,i);
	cvValue |= 1<<i;
	}
	currents[i]=current;
	//baseCurrents[i] = baseCurrent;
	}
	
	response.cvValue=cvValue;
	//state->operatingState = LEAVE_SERVICE_MODE;
	return response;*/
}

void setupReadCVBit(dccTransmitionState_t* state){
	
	
	if(serviceModeInfo.bitBeingProcessed > 0){
		//the current should read to tell what the last bit was
		uint8_t current = getAvgProgTrackCurrent();
		if(current > serviceModeInfo.baseCurrent && current - serviceModeInfo.baseCurrent > ACK_SAMPLE_THRESHOLD){
			serviceModeInfo.cvValue |= 1 << (serviceModeInfo.bitBeingProcessed-1);
		}
	}
	if(serviceModeInfo.bitBeingProcessed <8){
		//keep on reading, still more to collect
		serviceModeInfo.state = READ_BYTE_READ_BIT;
		}else{
		//now finished reading, time to stop
		state->operatingState=LEAVE_SERVICE_MODE;
		insertResetPacket(state,true);
		//TODO probably shouldn't be transmitting in this 'thread'
		transmitReadResult(serviceModeInfo.CVBeingProcessed,serviceModeInfo.cvValue,serviceModeInfo.callback,serviceModeInfo.callbacksub,true);
		return;
	}
	
	insertReadBitUsingDirectModePackets(state,serviceModeInfo.CVBeingProcessed,serviceModeInfo.bitBeingProcessed);
	
	serviceModeInfo.bitBeingProcessed++;
	
}

/*
* direct mode service mode to set the address CV
*
* returns if verify successful
*
* cv is 10bits
*/
void setCVwithDirectMode(dccTransmitionState_t* state, uint16_t cv, uint8_t newValue) {
	/*_delay_ms(500);
	setDataLED();
	_delay_ms(500);
	setIdleLED();
	_delay_ms(500);
	setDataLED();
	_delay_ms(500);
	setIdleLED();*/

	uint8_t i;
	dccPacket_t *packet;
	cvReadResponse_t response;
	/*
	waitForSafeToInsert(state);

	//at least three reset packets with long preamble
	for (i = 0; i < 10; i++) {//formerlly 5
	insertResetPacket(state, true);
	}*/
	enterServiceMode(state);

	for (i = 0; i < 15; i++) {
		packet = getInsertPacketPointer(state);
		/*long-preamble 0 0111CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
		two bit address (AA) in the first data byte being the most significant bits of the CV number.
		CC=10 Bit Manipulation
		CC=01 Verify byte
		CC=11 Write byte
		*/
		//cv '1' is actually cv 0
		cv--;
		
		//01111100 = 7c
		packet->address = 0b01111100 | ((cv >> 8) & 0b11); //write to CV, with the 2 MSB of CV number
		packet->data[0] = cv & 0xff; //lowest 8 bits of cv
		packet->data[1] = newValue;
		packet->dataBytes = 2;
		packet->longPreamble = true;
	}

	//at least 6 reset packets
	for (i = 0; i < 16; i++) {//was 8
		insertResetPacket(state, true);
	}
	//largely copied from DCC++ PacketRegister.cpp
	uint8_t baseCurrent = getAvgProgTrackCurrent();
	
	//TODO get base level of current reading before this, then request verification and wait until x many packets left in buffer before reading current again!
	
	for(i = 0;i<3;i++){
		insertResetPacket(state, true);
	}
	
	for (i = 0; i < 5; i++) {
		packet = getInsertPacketPointer(state);
		/*long-preamble 0 0111CCAA 0 AAAAAAAA 0 DDDDDDDD 0 EEEEEEEE 1
		two bit address (AA) in the first data byte being the most significant bits of the CV number.
		CC=10 Bit Manipulation
		CC=01 Verify byte
		CC=11 Write byte
		*/
		//cv '1' is actually cv 0
		cv--;
		
		packet->address = 0x74 | ((cv >> 8) & 0b11); //re-verify entire byte
		packet->data[0] = cv & 0xff; //lowest 8 bits of cv
		packet->data[1] = newValue;
		packet->dataBytes = 2;
		packet->longPreamble = true;
	}
	
	for(i = 0;i<2;i++){
		insertResetPacket(state, true);
	}
	
	//block until we've transmitted 4 of the verify packets
	while(getPacketsInBuffer(state) > 3);
	uint8_t current = getAvgProgTrackCurrent();
	
	//a bit of a hack that will result in the power to the track being cut off briefly:
	state->operatingState = LEAVE_SERVICE_MODE;
	
	response.success = false;
	
	if(current > baseCurrent && current - baseCurrent > ACK_SAMPLE_THRESHOLD){
		//success!
		response.success = true;
		response.cvValue = newValue;
	}
	
	
	//return response;
}

/*
* Use address-only mode to set an address
*/
bool setAddress(dccTransmitionState_t* state,  uint8_t newAddress) {
	//return setCVwithDirectMode(1, newAddress);
	uint8_t i;
	dccPacket_t *packet;
	
	/*waitForSafeToInsert(state);
	
	//3 or more Reset Packets
	for (i = 0; i < 5; i++) {
	insertResetPacket(state, true);
	}*/
	enterServiceMode(state);
	
	//5 or more Page-Preset-packets
	for (i = 0; i < 10; i++) {
		insertPagePresetPacket(state, true);
	}
	
	//6 or more Page Preset or Reset packets (Decoder-Recovery-Time from write to Page Register)
	for (i = 0; i < 9; i++) {
		insertResetPacket(state, true);
	}
	
	//Optional Power Off Followed by Power-On-Cycle
	
	//3 or more Reset Packets
	for (i = 0; i < 5; i++) {
		insertResetPacket(state, true);
	}
	
	//or 5 or more Writes to CV #1
	for (i = 0; i < 10; i++) {
		packet = getInsertPacketPointer(state);
		
		/*
		Instructions packets using Address-Only Mode are 3 byte packets of the format:
		long-preamble 0 0111C000 0 0DDDDDDD 0 EEEEEEEE 1
		
		c=0 for verify
		c=1 for write

		*/
		packet->address = 0b01111000;
		packet->data[0] = newAddress & 0b01111111;
		packet->dataBytes = 1;
		packet->longPreamble = true;
	}
	
	//10 or more identical Write or Reset packets (Decoder-Recovery-Time
	for (i = 0; i < 15; i++) {
		insertResetPacket(state, true);
	}
	
	//Optional Power Off
	//a bit of a hack that will result in the power to the track being cut off briefly:
	state->operatingState = LEAVE_SERVICE_MODE;
	return true;
	
	
}

/*
* Turn on or off front light (this is all atm, need to understand a bit more before implementing other features)
*/
void insertLightsPacket(dccTransmitionState_t* state,uint8_t address, bool on) {
	/*
	send a Function Group One Instruction
	The format of this instruction is 100DDDDD
	see RP-9.2.1 Extended Packet Format
	*/
	dccPacket_t *nextPacket = getInsertPacketPointer(state);
	nextPacket->address = address;
	nextPacket->data[0] = 0b10000000;

	if (on) {
		//the FL bit
		//note: "If Bit 1 of CV#29 has a value of one (1), then bit 4 controls function FL, otherwise bit 4 has no meaning."
		nextPacket->data[0] |= 0b10010000;
	}
	nextPacket->dataBytes = 1;
	nextPacket->longPreamble = false;
}

/*
* Speed is 3-28 (in 14step mode the LSB is ignored) in 14 adn 28 speedmodes
Speed is 2-127 in 128 speedmode
speed 0 is stop
speed 1,2 is emergency stop
*/
void insertSpeedPacket(dccTransmitionState_t* state, uint8_t address, uint8_t speed, bool forwards, uint8_t mode) {
	dccPacket_t *nextPacket = getInsertPacketPointer(state);
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
void waitForSafeToInsert(dccTransmitionState_t* state) {

	while (!state->safeToInsert);
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
		insertLightsPacket(&mainTrackState,address, true);
		
		
		
		//wait for it to be safe to insert a new packet
		while (!mainTrackState.safeToInsert);
		//now safe!
		(demoState)++;
		switch (demoState) {
			case 0:
			//go forwards!
			//USART_Transmit('f');
			for (i = 0; i < DUPLICATION; i++) {
				//insertSpeedPacket(address, 15, true, SPEEDMODE_14STEP);
				insertSpeedPacket(&mainTrackState, address, 100, true, SPEEDMODE_128STEP);
			}
			break;
			case 3:
			//USART_Transmit('\n');
			demoState = -1;
			case 1:
			//stop
			//USART_Transmit('s');
			for (i = 0; i < DUPLICATION; i++) {
				insertSpeedPacket(&mainTrackState, address, 0, true, SPEEDMODE_128STEP);
			}
			break;
			case 2:
			//go backwards!
			//USART_Transmit('b');
			for (i = 0; i < DUPLICATION; i++) {
				insertSpeedPacket(&mainTrackState, address, 100, false, SPEEDMODE_128STEP);
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
		Clrb(DCC_PORT, DCC_MAIN_TRACK_OUT);
		Setb(DCC_PORT, DCC_MAIN_TRACK_ENABLE);
		
		printADCValue();
		
		_delay_ms(DC_DELAY);
		//Setb(DCC_PORT, DCC_MAIN_TRACK_OUT);
		Clrb(DCC_PORT, DCC_MAIN_TRACK_ENABLE);

		printADCValue();

		_delay_ms(DC_DELAY);
		Setb(DCC_PORT, DCC_MAIN_TRACK_OUT);
		Setb(DCC_PORT, DCC_MAIN_TRACK_ENABLE);

		printADCValue();

		_delay_ms(DC_DELAY);
		//Setb(DCC_PORT, DCC_MAIN_TRACK_OUT);
		Clrb(DCC_PORT, DCC_MAIN_TRACK_ENABLE);
		
		printADCValue();
		
	}
}

/*
* Return a pointer to the current packet being transmitted in the packet buffer
*/
//inline

dccPacket_t *getCurrentPacket(dccTransmitionState_t* state) {
	return &(state->packetBuffer[state->transmittingPacket]);
}

/************************************************************************/
/* functions for turning the LEDs on and off. SetSerivceLED should only be used by the programming track, and doens't care about data and idle
data and idle don't carea bout the programming track, so leave service mode LED alone                                                                     */
/************************************************************************/
void setDataLED() {
	Setb(LED_PORT, LED_DATA);
	//Clrb(LED_PORT, LED_SERVICE_MODE);
	Clrb(LED_PORT, LED_IDLE);
}

void setServiceLED() {
	Setb(LED_PORT, LED_SERVICE_MODE);
	//Clrb(LED_PORT, LED_DATA);
	//Clrb(LED_PORT, LED_IDLE);
}

void clearServiceLED(){
	Clrb(LED_PORT, LED_SERVICE_MODE);
}

void setIdleLED() {
	Setb(LED_PORT, LED_IDLE);
	//Clrb(LED_PORT, LED_SERVICE_MODE);
	Clrb(LED_PORT, LED_DATA);
}

/*
* The packet buffer has just run out, fill it with something depending on what state we're in
*/
void fillPacketBuffer(dccTransmitionState_t* state) {
	switch (state->operatingState) {
		case OPERATIONS_MODE:
		//normal running mode, we just want to keep sending out idle packets
		//TODO here is where more inteligent logic about which commands to prioritise can go
		insertIdlePacket(state, false);
		setIdleLED();
		break;
		case LEAVE_SERVICE_MODE:
		//power off the programming track
		Clrb(DCC_PORT, DCC_PROG_TRACK_ENABLE);
		state->operatingState = OFF;
		clearServiceLED();
		break;
		case SERVICE_MODE:
		//insertResetPacket(state, true);
		//note, been lazy about re-assigning state inside the switch statement, so don't re-adjust order without being careful - or does this not matter in c?
		switch(serviceModeInfo.state){
			case READ_BYTE_READ_BIT:
			setupReadCVBit(state);
			break;
			case READ_BYTE_SETUP:
			//finished running through the packets to get the decoder into service mode, now we should request stuff
			//record what the base current was while stuff had been going on TODO calculate actually how long the setup takes and how long the avg current is calculated over
			serviceModeInfo.baseCurrent = getAvgProgTrackCurrent();
			setupReadCVBit(state);
			break;
		}
		break;
	}
}

/**
* we've reached the end of transmitting a bit, need to set up the state to transmit the next bit

NOTE transmittedBits is the bits that *will* have been transmitted after this function has returned


TODO - THE ADDRESS CAN/SHOULD JUST BE PART OF THE DATA, then it's just preamble + various data bytes + error detection

the error detection byte is all the data + Address xored togehter, furthering the
idea that address shouldn't be a special case


thought - the spec does differentiate between data and address, might it be easier to comprehend if I also do?
*/
uint8_t determineNextBit(dccTransmitionState_t* state) {
	state->safeToInsert = false;
	dccPacket_t *currentPacket = getCurrentPacket(state);
	switch (state->transmitPacketState) {
		case PREAMBLE:
		//only allow inserting new packets while transmitting preamble
		//in the hope that an interrupt will never try and add an idle packet at the same time as the main thread is adding new packets
		state->safeToInsert = true;
		state->transmittedBits++;
		//long preamble is for service mode packets
		if (state->transmittedBits >= (currentPacket->longPreamble ? LONG_PREAMBLE_LENGTH : PREAMBLE_LENGTH)) {
			//this is the last bit of the preamble
			state->transmitPacketState = PACKET_START_BIT;
		}
		return ONE_HIGH;
		break;
		case PACKET_START_BIT:
		//transmitting the packet start bit, the next state will be the address
		state->transmittedBits = 0;
		state->transmitPacketState = ADDRESS;
		return ZERO_HIGH1;
		break;
		case ADDRESS:
		state->transmittedBits++;

		if (state->transmittedBits >= 8) {
			//this is the last bit of the address, which state next?
			if (currentPacket->dataBytes > 0) {
				//there is data to transmit
				//start building up the error detection byte
				state->errorDetectionByte = currentPacket->address;
				state->transmittingDataByte = 0;
				state->transmitPacketState = DATA_START_BIT;
				} else {
				//no data to transmit
				state->transmitPacketState = END_BIT;
			}
		}

		//address data to transmit
		if (currentPacket->address & (1 << (8 - state->transmittedBits))) {
			//next MSB of address is 1
			return ONE_HIGH;
			} else {
			return ZERO_HIGH1;
		}

		break;
		case DATA_START_BIT:
		state->transmittedBits = 0;
		//this will always be 1 based, since it gets set to zero at the end of address
		state->transmittingDataByte++;
		state->transmitPacketState = DATA;
		return ZERO_HIGH1;
		break;
		case DATA:
		state->transmittedBits++;

		if (state->transmittedBits >= 8) {
			//reached end of this data byte
			//xor with error detection byte as per spec
			//I'm fairly certain that every data byte (& address byte) should be xored together to make the final error detection byte
			state->errorDetectionByte ^= currentPacket->data[state->transmittingDataByte - 1];
			if (currentPacket->dataBytes > state->transmittingDataByte) {
				//more data bytes to transmit
				state->transmitPacketState = DATA_START_BIT;
				} else {
				//no more to transmit, start the error correction byte
				state->transmitPacketState = ERROR_START_BIT;
			}
		}
		//still data bits to transmit
		if (currentPacket->data[state->transmittingDataByte - 1] & (1 << (8 -state-> transmittedBits))) {
			//jnext MSB of address is 1
			return ONE_HIGH;
			} else {
			return ZERO_HIGH1;
		}

		break;
		case ERROR_START_BIT:
		state->transmittedBits = 0;
		//next transmit the error detection byte
		state->transmitPacketState = ERROR_DETECTION;

		return ZERO_HIGH1;
		break;
		case ERROR_DETECTION:
		state->transmittedBits++;

		if (state->transmittedBits >= 8) {
			//reached end of this error detection byte
			state->transmitPacketState = END_BIT;

		}
		//still error detection bits to transmit
		if (state->errorDetectionByte & (1 << (8 - state->transmittedBits))) {
			//next MSB of error detect is 1
			return ONE_HIGH;
			} else {
			return ZERO_HIGH1;
		}

		break;
		case END_BIT:
		//the end bit can form part of the next preamble
		default:
		state->transmittedBits = 0;
		state->packetsInBuffer--;
		state->transmitPacketState = PREAMBLE;
		if (state->packetsInBuffer <= 0) {
			//no more packets in buffer, add an idle packet
			//the main loop will handle adding other packets
			//packetsInBuffer = 1;
			//insertIdlePacket(transmittingPacket);

			//find what packet to put in next (usualy an idle packet)
			fillPacketBuffer(state);
			} else {
			state->transmittingPacket++;
			state->transmittingPacket %= PACKET_BUFFER_SIZE;

			//detect what the next packet type is and set LED accordingly
			if (state->packetBuffer[state->transmittingPacket].longPreamble || state->serviceModeOnly) {
				//TODO improve this logic so the service mode LED is only on when we're actually doing something in service mode
				if(state->operatingState!=OFF){
					setServiceLED();
				}
				} else if (state->packetBuffer[state->transmittingPacket].address == 0xff) {
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
/* cut all high power outputs in a hurry                                */
/************************************************************************/
void emergencyCutPower(bool mainTrack){
	if(mainTrack){
		Clrb(DCC_PORT, DCC_MAIN_TRACK_ENABLE);
		}else{
		//programming track
		Clrb(DCC_PORT, DCC_PROG_TRACK_ENABLE);
	}
	Setb(LED_PORT, LED_OVERCURRENT);
}

/************************************************************************/
/* the state has all been set, set the correct outputs                  */
/************************************************************************/
inline void setOutputsFromInterrupt(dccTransmitionState_t* state){
	
	//output the right bit
	switch (state->bitState) {
		case ONE_HIGH:
		case ZERO_HIGH1:
		case ZERO_HIGH2:
		Setb(DCC_PORT, state->outputPin);
		break;
		case ONE_LOW:
		case ZERO_LOW1:
		case ZERO_LOW2:
		Clrb(DCC_PORT, state->outputPin);
		break;
	}
	
}

inline void updateNextOutput(dccTransmitionState_t* state){
	//proceed to output the rest of this bit, or work out what the next bit is
	switch (state->bitState) {
		case ONE_HIGH:
		state->bitState = ONE_LOW;
		break;
		case ZERO_HIGH1:
		state->bitState = ZERO_HIGH2;
		break;
		case ZERO_HIGH2:
		state->bitState = ZERO_LOW1;
		break;
		case ZERO_LOW1:
		state->bitState = ZERO_LOW2;
		break;

		case ONE_LOW:
		case ZERO_LOW2:
		default:
		//need the next bit!
		state->bitState = determineNextBit(state);
		break;
	}
}

#ifdef DEBUG_LED_FLASH
uint16_t debugledFlash = 0;
#endif
/************************************************************************/
/* Interrupt which is run every 58us                                    */
/************************************************************************/
ISR(TIMER0_COMPA_vect) {
	if (mainTrackCurrent > MAX_CURRENT){
		emergencyCutPower(true);
		//return;
	}
	if(progTrackCurrent > MAX_PROG_CURRENT){
		emergencyCutPower(false);
		//return;
	}
	setOutputsFromInterrupt(&mainTrackState);
	setOutputsFromInterrupt(&progTrackState);
	
	updateNextOutput(&mainTrackState);
	updateNextOutput(&progTrackState);
	
}