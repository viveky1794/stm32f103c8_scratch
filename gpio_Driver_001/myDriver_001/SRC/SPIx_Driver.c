/*
 * SPIx_Driver.c
 *
 *  Created on: Feb 14, 2020
 *      Author: vivek
 */

#include "SPIx_Driver.h"
#include "clock_Driver.h"


/* ****************************************************************************************
 * @FUN			: SPIx_Init
 * @Brief		: This API takes care of all Initialisions including RCC AHB and APB2 BUS clock
 * @para		:
 *
 * @retval		:
 * @NOTE		: If SSM=0(Hardware Slave Select), SPE=1 then NSS pin is 0 automatically.
 *				  if  SSM=0(Hardware Slave Select), SPE=0 then NSS pin is 1 automatically.
 *				  All this will happen only when (Slave Select Output Enable )SSOE=1. This pin is control pin.
 */

void SPI1_Init(void)
{

	// 1 : Set Clock on 1 Mhz
		set_RCC_HSI_Clock_APB2_1MHz();
	// 2 : Enable APB2 Peripherials
		// 2.1 Enable Alternate Functionality
		// 2.2 Enable SPI1 Functionality
		// 2.3 Select SPI PORT and PIN
			// I am using SPI1 so it require Port A
			// Pin A4	-->	SPI1_NSS
			// Pin A5	--> SPI1_SCK
			// Pin A6	--> SPI1_MISO
			// Pin A7	--> SPI1_MOSI

			RCC->APB2ENR	|= RCC_APB2RSTR_AFIORST | RCC_APB2RSTR_SPI1RST | RCC_APB2RSTR_IOPARST;

			GPIO_Init(GPIOA,4,GPIO_Mode_Out_PP,GPIO_Speed_10MHz);//NSS
			GPIO_Init(GPIOA,5,GPIO_Mode_Out_PP,GPIO_Speed_10MHz);//SCK
			GPIO_Init(GPIOA,6,GPIO_Mode_IPU,GPIO_Speed_INPUT);//MISO
			GPIO_Init(GPIOA,7,GPIO_Mode_Out_PP,GPIO_Speed_10MHz);//MOSI


	// 3 : SPI Register
		// 3.1 SPI Conrtol Register
			// 2-line unidirectional data mode selected in Full Duplex
			// BIDIMODE = 0 and RXONLY = 0 ->> Full Duplex
			SPI1->CR1	&= ~SPI_CR1_BIDIMODE;
			SPI1->CR1	&= ~SPI_CR1_RXONLY;

			// 1: Output enabled (transmit-only mode)
			SPI1->CR1	|= SPI_CR1_BIDIOE;
			// Data fram format is 16 bit long
			SPI1->CR1	|= SPI_CR1_DFF;
			// Full Duplex by default

			// we are not setting SSM bit So SSI bit will not have effect.

			// Clock Polarity which is 0 in ideal state
			// clock Phase which is 1 means at leading edge data will transite and trailing edge data will capture
			SPI1->CR1	|= SPI_CR1_CPHA;
			// Select as Master
			SPI1->CR1	|= SPI_CR1_MSTR;
			// Software Slave Manegment SSM
			SPI1->CR1	|= SPI_CR1_SSM | SPI_CR1_SSI; //otherwise MODF will come and MSTR will reset.
			// baudrate is dafault which is APB2/2
			// Frame Fromat is MSB first
			//Enable SIP do at last
			SPI1->CR1	|= SPI_CR1_SPE; // This bit is not set as desired MODF occured.

			SPI1->CR2	|= SPI_CR2_SSOE;// @ref NOTES
}


void SPI_Deinit(void)
{
	while( Flag_Status(BSY) ==  BSY_FLAG);

	SPI1->CR1	&= ~SPI_CR1_SPE;

}

/* ***********************************************************************************
 * @Fun			:	Flag_Status
 * @Brief		:	This API gives various Flag Status which help developers in operations
 * @para		:	Flag name
 * 					# Receive Buffer
 * 					# Transmit Buffer
 * 					# Mode Fault
 * 					# Busy Status of SPI communication
 *
 * @NOTE		:	Master mode fault occurs when the master device has its NSS pin pulled low (in NSS
 *					hardware mode) or SSI bit low (in NSS software mode), this automatically sets the MODF
 *					bit. Master mode fault affects the SPI peripheral in the following ways:
 *					• The MODF bit is set and an SPI interrupt is generated if the ERRIE bit is set.
 *					• The SPE bit is cleared. This blocks all output from the device and disables the SPI
 *					interface.
 *					• The MSTR bit is cleared, thus forcing the device into slave mode.
 *					Use the following software sequence to clear the MODF bit:
 *					1. Make a read or write access to the SPI_SR register while the MODF bit is set.
 *					2. Then write to the SPI_CR1 register.
 * @retval		:	Flag status OR Error
 *
 */

uint8_t Flag_Status(uint8_t Flag)
{
	switch(Flag)
		{
		case RXNE:
					if( SPI1->SR & ( 1 << 0) )
						return ( RX_NOT_EMPTY);
					else
						return ( RX_EMPTY);
					break;
		case TXE:
				if( SPI1->SR & ( 1 << 1) )
					return ( TX_EMPTY);
				else
					return ( TX_NOT_EMPTY);
				break;

		case MODF:
				if( SPI1->SR & ( 1 << 5) )
					return ( MODE_FAULT );
				else
					return ( MODE_NOT_FAULT);
				break;
		case BSY:
				if( SPI1->SR & ( 1 << 7) )
					return ( BSY_FLAG );
				else
					return ( BSY_NOT_FLAG);
				break;

		}
return ERROR;
}

/* ****************************************************************************************
 * @FUN			: SPIx_ReadData
 * @Brief		: Reading data from SPI Data Registers
 *
 * @para		:
 * @retval		:
 *
 *
 */

uint16_t SPIx_ReadData()
{
	uint16_t Data=0;
	Data	= SPI1->DR & (0xFFFF);
	return  Data;
}


/* ****************************************************************************************
 * @FUN			: SPIx_Write
 * @Brief		: Writing Data to SPI Data Registers
 * 				  This is a blocking API utill all data is not transfered.
 *
 * @para_1		: Data needs to be send
 * @para_2		: Length of Data
 * @retval		:
 *
 *
 */
uint16_t SPIx_Write(uint16_t *Data,uint8_t len)
{
	do
	{
		// halt here if tx buffer is not empty
		while( ! Flag_Status(TXE) );

		SPI1->DR |= *Data;
		len -= 2;//beacuse our data is in two byte
	}while(len);

	return 1;
}
