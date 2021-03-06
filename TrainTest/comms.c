/*
* comms.c
*
* Created: 22/03/2014 14:58:08
*  Author: Luke
*/

#include "comms.h"
#include <util/crc16.h>


/************************************************************************/
/* read in a byte from the UART buffer, blocking until available        */
/* THIS IGNORES ANY ERRORS TODO NOT IGNORE ERROR                        */
/************************************************************************/
uint8_t readByteBlocking(){
	uint16_t c;
	for(;;)
	{
		/*
		* Get received character from ringbuffer
		* uart_getc() returns in the lower byte the received character and
		* in the higher byte (bitmask) the last receive error
		* UART_NO_DATA is returned when no data is available.
		*
		*/
		c = uart_getc();
		if ( c & UART_NO_DATA )
		{
			/*
			* no data available from UART
			*/
		}
		else
		{
			/*
			* new data available from UART
			* check for Frame or Overrun error
			*/
			if ( c & UART_FRAME_ERROR )
			{
				/* Framing Error detected, i.e no stop bit detected */
				//uart_puts_P("UART Frame Error: ");
			}
			if ( c & UART_OVERRUN_ERROR )
			{
				/*
				* Overrun, a character already present in the UART UDR register was
				* not read by the interrupt handler before the next character arrived,
				* one or more received characters have been dropped
				*/
				//uart_puts_P("UART Overrun Error: ");
			}
			if ( c & UART_BUFFER_OVERFLOW )
			{
				/*
				* We are not reading the receive buffer fast enough,
				* one or more received character have been dropped
				*/
				//uart_puts_P("Buffer overflow error: ");
			}
			/*
			* send received character back
			*/
			return (uint8_t)c;
		}
	}
}


/*
* Keep reading UART until we've finished a sync message
*/
void readUntilSync() {
	static volatile uint32_t syncBytes = SYNC_INT;
	uint8_t * syncPointer = (uint8_t*) & syncBytes;

	uint8_t testBytes[NUM_SYNC_BYTES];

	for(uint8_t i=0;i<NUM_SYNC_BYTES;i++){
		testBytes[i] = readByteBlocking();
	}
	
	//while (true) {
	for(uint8_t j=0;j<NUM_SYNC_BYTES;j++){
		if (testBytes[j] != *(syncPointer + j)) {
			//FAILED. get rid of the first byte, read one more new byte and try again.

			//shuffle them all left by one
			for(uint8_t k=0;k<NUM_SYNC_BYTES-1;k++){
				testBytes[k] = testBytes[k+1];
			}
			transmitCommsDebug(1);
			testBytes[NUM_SYNC_BYTES-1] = readByteBlocking();
			//go back to the beginning of the while loop with the new test bytes
			j=0;
		}
	}
	//if we've got here then we went through the loop NUM_SYNC_BYTES times without one being wrong!
	return;
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
		debugBuffer[i] = readByteBlocking();
	}
	#endif
	//sync up!
	readUntilSync();

	for (i = 0; i<sizeof(message_t); i++) {
		uint8_t in =  readByteBlocking();
		*messagePointer = in;
		messagePointer++;
	}

	return message;
}


bool areSyncBytes(uint8_t* bytes){
	static volatile uint32_t syncBytes = SYNC_INT;
	uint8_t * syncPointer = (uint8_t*) & syncBytes;
	for(uint8_t j=0;j<NUM_SYNC_BYTES;j++){
		if (bytes[j] != *(syncPointer + j)) {
			return false;
		}
	}
	return true;
}



/************************************************************************/
/* Transmit the current size of the packet buffer back to the listener  */
/* also send current current draw and staet of power on main track      */
/************************************************************************/
void transmitStatus(uint8_t size, uint8_t currentDraw, bool trackPower){
	message_t message;
	
	message.commandType=RESPONSE_PACKET_BUFFER_SIZE;
	message.data.statusMessageData.packetsInBuffer = size;
	message.data.statusMessageData.mainTrackCurrent =currentDraw;
	message.data.statusMessageData.mainTrackPowered = trackPower;
	message.crc = calculateCRC(&message);
	
	transmitMessage((uint8_t*)&message);
}


void transmitCommsDebug(uint8_t type){
	message_t message;
	
	
	
	message.commandType=RESPONSE_COMMS_ERROR;
	message.data.genericMessageData.data[0]=type;
	message.crc = calculateCRC(&message);
	
	transmitMessage((uint8_t*)&message);
}

void transmitCurrentDraw(uint8_t current){
	message_t message;
	
	message.commandType=REPONSE_CURRENT;
	message.data.currentDrawData.currentDraw=current;
	message.crc = calculateCRC(&message);
	
	transmitMessage((uint8_t*)&message);
}

void transmitCVResult(uint16_t cv, uint8_t cvValue,uint16_t callback, uint16_t callbackSub,bool success){
	message_t message;
	//TODO
	message.commandType=RESPONSE_CV;
	//message.commandType= responseType;
	message.data.cvResponseData.cvValue=cvValue;
	message.data.cvResponseData.callback=callback;
	message.data.cvResponseData.callbackSub=callbackSub;
	message.data.cvResponseData.success = success;
	message.data.cvResponseData.cv=cv;
	//message.data.cvResponseData.responseType = responseType;
	message.crc = calculateCRC(&message);
	
	transmitMessage((uint8_t*)&message);
}

/************************************************************************/
/* Transmit a message over UART                                         */
/************************************************************************/
void transmitMessage(uint8_t* messagePointer){
	static volatile uint32_t syncBytes = SYNC_INT;
	//send the sync bytes
	uint8_t * syncPointer = (uint8_t*) & syncBytes;
	for(uint8_t i=0;i<NUM_SYNC_BYTES;i++){
		uart_putc(*(syncPointer + i));
	}
	
	//send the message
	//data + command + crc
	for (uint8_t i = 0; i<MAX_MESSAGE_DATA_BYTES+2; i++) {//two extra bytes for command type and CRC
		uart_putc(*messagePointer);
		messagePointer++;
	}
	
}

bool checkCRC(message_t* message){
	
	uint8_t crc = calculateCRC(message);
	return crc == message->crc;
	
}

uint8_t calculateCRC(message_t* message){
	uint8_t* messagePtr = (uint8_t*)message;
	
	uint8_t crc = 0;
	//data + command
	for(uint8_t i=0;i<MAX_MESSAGE_DATA_BYTES+1;i++){//one extra byte for the command type
		
		crc = _crc_ibutton_update(crc,messagePtr[i]);
	}
	return crc;
}

void processMessage(message_t* message){
	dccPacket_t *packet;
	uint8_t i;
	//uint8_t cvValue;
	cvResponse_t cvResponse;
	//bool success;
	
	if(!checkCRC(message)){
		transmitCommsDebug(4);
		return;
	}
	
	switch (message->commandType) {

		case COMMAND_PROGRAMME_DIRECT_BYTE:
		/*
		* Pop into programming mode, write the CV, pop back out.
		*/
		cvResponse = setCVwithDirectMode( message->data.directByteCVMessageData.cv, message->data.directByteCVMessageData.newValue);
		for(i=0;i<3;i++){
			transmitCVResult( message->data.directByteCVMessageData.cv, message->data.directByteCVMessageData.newValue, message->data.directByteCVMessageData.callback,message->data.directByteCVMessageData.callbackSub, cvResponse.success);
		}
		break;
		case COMMAND_READ_CV:
		//cvResponse = 
		cvResponse = readCVWithDirectMode(message->data.directByteCVMessageData.cv);
		//I do not understand why, but often the first attempt to sent a message is lost. TODO find out why! this feels like a major issue somewhere, repetition is just a work around
		for(i=0;i<3;i++){
			transmitCVResult( message->data.directByteCVMessageData.cv,cvResponse.cvValue, message->data.directByteCVMessageData.callback,message->data.directByteCVMessageData.callbackSub, cvResponse.success);
		}
		break;
		case COMMAND_PROGRAMME_DIRECT_BIT:
		cvResponse = setCVBitwithDirectMode(message->data.directByteCVMessageData.bit, message->data.directByteCVMessageData.newValue, message->data.directByteCVMessageData.cv);
		for(i=0;i<3;i++){
			transmitCVResult( message->data.directByteCVMessageData.cv, message->data.directByteCVMessageData.newValue, message->data.directByteCVMessageData.callback,message->data.directByteCVMessageData.callbackSub, cvResponse.success);
		}
		break;
		case COMMAND_PROGRAMME_ADDRESS:
		//this old instruction doesn't do any verification, but we need to return something to let the server know we've finished
		cvResponse.success = setAddress(message->data.newAddressMessageData.newAddress);
		//for(i=0;i<3;i++){
		//	transmitCVResult( 1, message->data.newAddressMessageData.newAddress, message->data.directByteCVMessageData.callback,message->data.directByteCVMessageData.callbackSub, cvResponse.success);
		//}
		//TODO - need to modify to have callbacks to do this properly
		break;
		case COMMAND_OPERATIONS_MODE_PACKET:
		
		for (i = 0; i < message->data.opsModePacketMessageData.repeat; i++) {
			waitForSafeToInsert();
			packet = getInsertPacketPointer();
			//address is actually just the first data byte as far as DCC/JMRI is concerned, it's *normally* address which is why I called it hta to begin with
			packet->address = message->data.opsModePacketMessageData.address;
			//comms protocol is assumign that address is part of the data, so subtract one from this until internally
			//address is subsumed into data
			//also remove one because JMRI transmits the error detection packet, which *we* generate ourselves!
			//todo compare JMRI's error detection packet with what we calculate - possibly not necessary because we generate CRC of whole message?
			packet->dataBytes = message->data.opsModePacketMessageData.dataBytes - 2;
			//error detection should be generated same as JMRI's
			memcpy(packet->data, message->data.opsModePacketMessageData.data, message->data.opsModePacketMessageData.dataBytes);
			//this is only used for main track operations, all service mode requests have all their packets generated here, not at JMRI's end
			packet->longPreamble = false;
		}
		break;
		/*case COMMAND_SET_SPEED:
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
		break;*/
		/*case COMMAND_ENTER_SERVICE_MODE:
		enterServiceMode();

		break;*/
		case COMMAND_REQUEST_BUFFER_SIZE:
		//nothing to actually do, this is done in response to every single message atm
		break;
		case COMMAND_REQUEST_CURRENT:
		//this is now also in the general status response to every message
		//		transmitCurrentDraw(adc_read());
		break;
		case COMMAND_SET_POWER:
		//turns the power on or off for either the main track or programming track
		if(message->data.trackPoweredData.whichTrack==MAIN_TRACK){
			setMainTrackPower(message->data.trackPoweredData.powered);
		}else
		if(message->data.trackPoweredData.whichTrack==PROG_TRACK){
			setProgTrackPower(message->data.trackPoweredData.powered);
		}
		break;
		case COMMAND_SHIFT_REGISTER_DATA:
		//this is data that will be sent out over SPI for the LED drivers/point motors
		//once all the data has been collected it will be shifted out and a latch pin raised
		setShiftRegisterData(message->data.shiftRegisterData.startByte,message->data.shiftRegisterData.data);
		
		break;
		case COMMAND_SET_SHIFT_REGISTER_LENGTH:
		resetShiftRegister(message->data.shiftRegisterLengthData.length);
		break;
		case COMMAND_OUTPUT_SHIFT_REGISTER:
		outputShiftRegister();
		break;
		default:
		transmitCommsDebug(2);
		break;
	}
}



uint8_t inputBuffer[INPUT_BUFFER_SIZE];
uint8_t inputBufferEndPosition = 0;
uint8_t inputBufferStartPosition = 0;

void bufferInput(void){
	uint16_t c;
	
	c = uart_getc();
	if ( c & UART_NO_DATA )
	{
		/*
		* no data available from UART
		*/
	}
	else
	{
		/*
		* new data available from UART
		* check for Frame or Overrun error
		*/
		if ( c & UART_FRAME_ERROR )
		{
			/* Framing Error detected, i.e no stop bit detected */
			//uart_puts_P("UART Frame Error: ");
		}
		if ( c & UART_OVERRUN_ERROR )
		{
			/*
			* Overrun, a character already present in the UART UDR register was
			* not read by the interrupt handler before the next character arrived,
			* one or more received characters have been dropped
			*/
			//uart_puts_P("UART Overrun Error: ");
		}
		if ( c & UART_BUFFER_OVERFLOW )
		{
			/*
			* We are not reading the receive buffer fast enough,
			* one or more received character have been dropped
			*/
			//uart_puts_P("Buffer overflow error: ");
		}
		/*
		* c is available!
		*/
		//return (uint8_t)c;
		inputBuffer[inputBufferEndPosition] = (uint8_t)c;
		inputBufferEndPosition++;
	}
}

void processInput(void){
	

	
	if(inputBufferEndPosition - inputBufferStartPosition > FULL_MESSAGE_LENGTH){
		if(!areSyncBytes(&inputBuffer[inputBufferStartPosition])){
			inputBufferStartPosition++;
			}else{
			transmitCommsDebug(3);
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
		transmitCommsDebug(1);
	}
	
}



#ifdef commsworking
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
			setCVwithDirectMode(message.data.programmeDirectByteMessageData.cvValue, message.data.programmeDirectByteMessageData.newValue);
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
#endif