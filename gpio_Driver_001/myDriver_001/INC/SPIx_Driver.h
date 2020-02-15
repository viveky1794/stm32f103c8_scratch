/*
 * SPIx_Driver.h
 *
 *  Created on: Feb 14, 2020
 *      Author: vivek
 */

#ifndef INC_SPIX_DRIVER_H_
#define INC_SPIX_DRIVER_H_


/* *********************************************************************************************
 * The serial peripheral interface (SPI) allows half/ full-duplex, synchronous, serial			//
 * communication with external devices. The interface can be configured as the master and in	//
 * this case it provides the communication clock (SCK) to the external slave device. The		//
 * interface is also capable of operating in multimaster configuration.							//
 * 																								//

 ************************************* SPI Features *******************************************
	• Full-Duplex synchronous transfers on three lines											//
	• Simplex synchronous transfers on two lines with or without a bidirectional data line		//
	• 8- or 16-bit transfer frame format selection												//
	• Master or slave operation																	//
	• Multimaster mode capability																//
	• 8 master mode baud rate Prescalers (fPCLK/2 max.)											//
	• Slave mode frequency (fPCLK/2 max)														//
	• Faster communication for both master and slave											//
	• NSS management by hardware or software for both master and slave: dynamic change			//
		of master/slave operations																//
	• Programmable clock polarity and phase														//
	• Programmable data order with MSB-first or LSB-first shifting								//
	• Dedicated transmission and reception flags with interrupt capability						//
	• SPI bus busy status flag																	//
	• Hardware CRC feature for reliable communication:											//
	– CRC value can be transmitted as last byte in Tx mode										//
	– Automatic CRC error checking for last received byte										//
	• Master mode fault, overrun and CRC error flags with interrupt capability					//
	• 1-byte transmission and reception buffer with DMA capability: Tx and Rx requests			//
***********************************************************************************************
	Usually, the SPI is connected to external devices through four pins:						//
																								//
	• MISO: Master In / Slave Out data. This pin can be used to transmit data in slave mode		//
	and receive data in master mode.															//
	• MOSI: Master Out / Slave In data. This pin can be used to transmit data in master			//
	mode and receive data in slave mode.														//
	• SCK: Serial Clock output for SPI masters and input for SPI slaves.						//
	• NSS: Slave select. This is an optional pin to select a slave device. This pin acts as a	//
	‘chip select’ to let the SPI master communicate with slaves individually and to avoid		//
	contention on the data lines. Slave NSS inputs can be driven by standard IO ports on		//
	the master device. The NSS pin may also be used as an output if enabled (SSOE bit)			//
	and driven low if the SPI is in master configuration. In this manner, all NSS pins from		//
	devices connected to the Master NSS pin see a low level and become slaves when				//
	they are configured in NSS hardware mode.													//

*/


/***************************** SPI and I2S registers ********************************************/

#include "stm32f103c8_MCU.h"


// USe Enum
#define RXNE	0
#define TXE		1
#define MODF	5
#define BSY		7

#define RX_EMPTY		0
#define TX_NOT_EMPTY	0
#define MODE_NOT_FAULT  0
#define BSY_NOT_FLAG	0

#define RX_NOT_EMPTY	1
#define TX_EMPTY		1
#define MODE_FAULT		1//bit 5
#define BSY_FLAG		1
#define ERROR			2


void SPI1_Init(void);


#endif /* INC_SPIX_DRIVER_H_ */
