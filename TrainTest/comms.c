/*
* comms.c
*
* Created: 22/03/2014 14:58:08
*  Author: Luke
*/

#include "comms.h"
uint32_t syncBytes = SYNC_INT;
/*
* Keep reading UART until we've finished a sync message
*/
void readUntilSync() {

	uint8_t * syncPointer = (uint8_t*) & syncBytes;
	//uint8_t syncPos = 0;
	//uint8_t in;
	//volatile uint8_t test;
	//uint8_t attempts = 0;

	uint8_t testBytes[NUM_SYNC_BYTES];

	for(uint8_t i=0;i<NUM_SYNC_BYTES;i++){
		testBytes[i] = USART_Receive();
	}
	


	//while (true) {
	for(uint8_t j=0;j<NUM_SYNC_BYTES;j++){
		if (testBytes[j] != *(syncPointer + j)) {
			//FAILED. get rid of the first byte, read one more new byte and try again.

			//shuffle them all left by one
			for(uint8_t k=0;k<NUM_SYNC_BYTES-1;k++){
				testBytes[k] = testBytes[k+1];
			}
			transmitCommsDebug();
			testBytes[NUM_SYNC_BYTES-1] = USART_Receive();
			//go back to the beginning of the while loop with the new test bytes
			j=0;
		}
	}
	//if we've got here then we went through the loop NUM_SYNC_BYTES times without one being wrong!
	return;
	//}
}

bool areSyncBytes(uint8_t* bytes){
	uint8_t * syncPointer = (uint8_t*) & syncBytes;
	for(uint8_t j=0;j<NUM_SYNC_BYTES;j++){
		if (bytes[j] != *(syncPointer + j)) {
			return false;
		}
	}
	return true;
}

/**
* Block waiting for a message over serial
*/
message_t readMessage(void) {
	uint8_t i;
	message_t message;

	uint8_t *messagePointer = (uint8_t *) & message;
	#ifdef DEBUGUART
	volatile uint8_t debugBuffer[128];
	
	for(i=0;i<128;i++){
		debugBuffer[i] = USART_Receive();
	}
	#endif
	//sync up!
	readUntilSync();

	for (i = 0; i<9; i++) {//
		uint8_t in = USART_Receive();
		*messagePointer = in;
		messagePointer++;
	}

	return message;
}

/************************************************************************/
/* Transmit the current size of the packet buffer back to the listener  */
/************************************************************************/
void transmitPacketBufferSize(uint8_t size){
	message_t message;
	
	message.commandType=RESPONSE_PACKET_BUFFER_SIZE;
	message.data.packetBufferSizeData.packetsInBuffer = size;
	
	transmitMessage((uint8_t*)&message);
}



void transmitCommsDebug(void){
	message_t message;
	
	message.commandType=RESPONSE_COMMS_ERROR;
	
	transmitMessage((uint8_t*)&message);
}

/************************************************************************/
/* Transmit a message over UART                                         */
/************************************************************************/
void transmitMessage(uint8_t* messagePointer){
	
	//send the sync bytes
	uint8_t * syncPointer = (uint8_t*) & syncBytes;
	for(uint8_t i=0;i<NUM_SYNC_BYTES;i++){
		USART_Transmit(*(syncPointer + i));
	}
	
	//send the message
	
	for (uint8_t i = 0; i<sizeof (message_t); i++) {
		USART_Transmit(*messagePointer);
		messagePointer++;
	}
	
}

uint8_t inputBuffer[INPUT_BUFFER_SIZE];
uint8_t inputBufferEndPosition = 0;
uint8_t inputBufferStartPosition = 0;

void bufferInput(void){
	if(USART_DataInAvailable()){
		inputBuffer[inputBufferEndPosition] = USART_ReceiveNonBlock();
		inputBufferEndPosition++;
	}
}

void processInput(void){
	

	
	if(inputBufferEndPosition - inputBufferStartPosition > FULL_MESSAGE_LENGTH){
		if(!areSyncBytes(&inputBuffer[inputBufferStartPosition])){
			inputBufferStartPosition++;
		}else{
			//we have a whole message in the buffer, with valid sync bytes at the start
			processMessage((message_t*)&inputBuffer[inputBufferStartPosition]);
			inputBufferStartPosition=0;
			inputBufferEndPosition=0;
		}
	}

	if(inputBufferEndPosition >= INPUT_BUFFER_SIZE-1){
		//this shouldn't happen, we've collected a lot of garbage (and possibly the beginning of a real message)
		//discard it all
		inputBufferStartPosition=0;
		inputBufferEndPosition=0;
		transmitCommsDebug();
	}
	
}


void processMessage(message_t* message){
	dccPacket_t *packet;
	uint8_t i;
	//USART_Transmit('r');
	switch (message->commandType) {

		case COMMAND_PROGRAMME_DIRECT_BYTE:
		/*
		* Pop into programming mode, write the CV, pop back out.
		*/
		setCVwithDirectMode(message->data.programmeDirectByteMessageData.cv, message->data.programmeDirectByteMessageData.newValue);
		break;
		case COMMAND_PROGRAMME_ADDRESS:
		setAddress(message->data.newAddressMessageData.newAddress);
		break;
		case COMMAND_OPERATIONS_MODE_PACKET:
		
		for (i = 0; i < message->data.opsModePacketMessageData.repeat; i++) {
			waitForSafeToInsert();
			//for(i=0;i< DUPLICATION;i++){
			packet = getInsertPacketPointer();
			//address is actually just the first data byte as far as DCC/JMRI is concerned, it's *normally* address which is why I called it hta to begin with
			packet->address = message->data.opsModePacketMessageData.address;
			//comms protocol is assumign that address is part of the data, so subtract one from this until internally
			//address is subsumed into data
			//also remove one because JMRI transmits the error detection packet, which *we* generate ourselves!
			packet->dataBytes = message->data.opsModePacketMessageData.dataBytes - 2;
			//error detection should be generated same as JMRI's
			//will this work?
			//packet->data=message->data.customPacketMessageData.data;
			memcpy(packet->data, message->data.opsModePacketMessageData.data, message->data.opsModePacketMessageData.dataBytes);
			//TODO will this need to change?
			packet->longPreamble = false;

			//insertSpeedPacket(message->address, 80, true, SPEEDMODE_128STEP);
		}
		break;
		case COMMAND_SET_SPEED:
		waitForSafeToInsert();
		for (i = 0; i < DUPLICATION; i++) {
			insertSpeedPacket(message->data.speedMessageData.address, message->data.speedMessageData.speed, message->data.speedMessageData.forwards, SPEEDMODE_128STEP);
		}
		break;
		case COMMAND_ENABLE_LIGHTS:
		waitForSafeToInsert();
		for (i = 0; i < DUPLICATION; i++) {
			insertLightsPacket(message->data.lightsMessageData.address, message->data.lightsMessageData.on);
		}
		break;
		case COMMAND_EMERGENCY_STOP:
		//needs testing
		waitForSafeToInsert();
		insertSpeedPacket(0, 1, false, SPEEDMODE_14STEP);
		break;
		case COMMAND_ENTER_SERVICE_MODE:
		enterServiceMode();

		break;
	}
}

/*
* Take control and just sit there processing input from uart
*/
void oldprocessInput(bool blocking) {
	do{
		message_t message = readMessage();
		dccPacket_t *packet;
		uint8_t i;
		//USART_Transmit('r');
		switch (message.commandType) {

			case COMMAND_PROGRAMME_DIRECT_BYTE:
			/*
			* Pop into programming mode, write the CV, pop back out.
			*/
			setCVwithDirectMode(message.data.programmeDirectByteMessageData.cv, message.data.programmeDirectByteMessageData.newValue);
			break;
			case COMMAND_PROGRAMME_ADDRESS:
			setAddress(message.data.newAddressMessageData.newAddress);
			break;
			case COMMAND_OPERATIONS_MODE_PACKET:
			
			for (i = 0; i < message.data.opsModePacketMessageData.repeat; i++) {
				waitForSafeToInsert();
				//for(i=0;i< DUPLICATION;i++){
				packet = getInsertPacketPointer();
				//address is actually just the first data byte as far as DCC/JMRI is concerned, it's *normally* address which is why I called it hta to begin with
				packet->address = message.data.opsModePacketMessageData.address;
				//comms protocol is assumign that address is part of the data, so subtract one from this until internally
				//address is subsumed into data
				//also remove one because JMRI transmits the error detection packet, which *we* generate ourselves!
				packet->dataBytes = message.data.opsModePacketMessageData.dataBytes - 2;
				//error detection should be generated same as JMRI's
				//will this work?
				//packet->data=message.data.customPacketMessageData.data;
				memcpy(packet->data, message.data.opsModePacketMessageData.data, message.data.opsModePacketMessageData.dataBytes);
				//TODO will this need to change?
				packet->longPreamble = false;

				//insertSpeedPacket(message.address, 80, true, SPEEDMODE_128STEP);
			}
			break;
			case COMMAND_SET_SPEED:
			waitForSafeToInsert();
			for (i = 0; i < DUPLICATION; i++) {
				insertSpeedPacket(message.data.speedMessageData.address, message.data.speedMessageData.speed, message.data.speedMessageData.forwards, SPEEDMODE_128STEP);
			}
			break;
			case COMMAND_ENABLE_LIGHTS:
			waitForSafeToInsert();
			for (i = 0; i < DUPLICATION; i++) {
				insertLightsPacket(message.data.lightsMessageData.address, message.data.lightsMessageData.on);
			}
			break;
			case COMMAND_EMERGENCY_STOP:
			//needs testing
			waitForSafeToInsert();
			insertSpeedPacket(0, 1, false, SPEEDMODE_14STEP);
			break;
			case COMMAND_ENTER_SERVICE_MODE:
			enterServiceMode();

			break;
		}
	}while(blocking);
}