/*
 * spi.h
 *
 * Created: 04/04/2017 14:22:38
 *  Author: Luke
 */ 


#ifndef SPI_H_
#define SPI_H_

#include "Include.h"

void spi_init(void);
void SPI_MasterTransmit(uint8_t cData);
void SPI_MasterTransmitArray(uint8_t* data, uint16_t length);

#define DDR_SPI DDRB
#define DD_MOSI DDB5
#define DD_SCK DDB7
#define DD_SS DDB4

#endif /* SPI_H_ */