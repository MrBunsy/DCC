/*
* spi.c
*
* Created: 04/04/2017 14:22:30
*  Author: Luke
*/

#include "spi.h"
/************************************************************************/
/*                                                                      */
/************************************************************************/
void spi_init(void){
	//to enable the SPI module, Power Reduction Serial Peripheral Interface bit in the Power Reduction Register (0.PRSPI0) must be written to '0'.
	Clrb(PRR0, PRSPI);
	
	//TLC5917: Clock input for data shift on rising edge
	//so I think we can leave CPOL=0, CPHA=0 as per default
	
	//from AVR datasheet
	/* Set MOSI and SCK output, all others input */
	DDR_SPI = (1<<DD_MOSI)|(1<<DD_SCK) | (1<<DD_SS);
	/* Enable SPI, Master, set clock rate fck/16 */
	SPCR = (1<<SPE)|(1<<MSTR)|(1<<SPR0);
	
	

}

/************************************************************************/
/* Write one byte out on SPI											*/
/* from the AVR datasheet                                               */
/************************************************************************/
void SPI_MasterTransmit(uint8_t cData)
{
	/* Start transmission */
	SPDR = cData;
	/* Wait for transmission complete */
	while(!(SPSR & (1<<SPIF)))
	;
}

void SPI_MasterTransmitArray(uint8_t* data, uint16_t length){
	for(uint16_t i=0;i<length;i++){
		SPI_MasterTransmit(data[i]);
	}
}