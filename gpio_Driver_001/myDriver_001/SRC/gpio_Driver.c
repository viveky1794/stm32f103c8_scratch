/*
 * gpio_Driver.c
 *
 *  Created on: Feb 12, 2020
 *      Author: vivek
 */

#include "gpio_Driver.h"

void GPIO_Init(		GPIO_TypeDef *pGPIOx,
					uint8_t Pin,
					uint8_t Mode,
					uint8_t Speed		)
{

	if(Mode == GPIO_Mode_Out_PP)
	{
		// 1: Enable Peripherial Clock
		if(pGPIOx == GPIOA)
		{
			Enable_RCC_Pheririal_BUS(IOPAEN);
		}else if(pGPIOx == GPIOB)
		{
			Enable_RCC_Pheririal_BUS(IOPBEN);
		}else if(pGPIOx == GPIOC)
		{
			Enable_RCC_Pheririal_BUS(IOPCEN);
		}

		if(Pin >= 8)
			Pin = Pin%8;

		// 2: Setting GPIO pin Speed
		pGPIOx->CRH |= Speed << (Pin*4) ;
		// 3 : Setting GPIO pin Mode(OUTPUT)
		pGPIOx->CRH |= Mode << ((Pin*4)+2);

	}else if(Mode == GPIO_Mode_IPU)
	{
		// 1: Enable Peripherial Clock
			if(pGPIOx == GPIOA)
			{
				Enable_RCC_Pheririal_BUS(IOPAEN);
			}else if(pGPIOx == GPIOB)
			{
				Enable_RCC_Pheririal_BUS(IOPBEN);
			}else if(pGPIOx == GPIOC)
			{
				Enable_RCC_Pheririal_BUS(IOPCEN);
			}

			if(Pin >= 8)
				Pin = Pin%8;

			// 2: Setting GPIO pin Speed
			pGPIOx->CRH |= Speed << (Pin*4) ;
			// 3 : Setting GPIO pin Mode(INPUT)
			pGPIOx->CRH &= (0x0 << 26);// it is required
			pGPIOx->CRH |= (0x8 << 24);

	}else if(Mode == GPIO_Mode_AF_PP)
	{
		// 1: Enable Peripherial Clock
		if(pGPIOx == GPIOA)
		{
			Enable_RCC_Pheririal_BUS(IOPAEN);
		}else if(pGPIOx == GPIOB)
		 {
			Enable_RCC_Pheririal_BUS(IOPBEN);
		 }else if(pGPIOx == GPIOC)
			{
				Enable_RCC_Pheririal_BUS(IOPCEN);
			}

		Enable_RCC_Pheririal_BUS(AFIO_EN);

		// 2 : Mask CRL[CFR and MODE] bits
		pGPIOx->CRL &= ~(0xF << Pin*4);
		// 3 : Set Speed bits[MODE] for Input[00]
		pGPIOx->CRL |= Speed << Pin*4;
		// 4 : Set CFR bits of CRL
		pGPIOx->CRL |= Mode << ( (Pin*4)+2);
		// 5 : Set EXTI5 Port B---GPIOB
		AFIO->EXTICR[1] |= 1<<4;
		// 6 : Set Interrupt
		EXTI->IMR  |= 1<<5;
		// 7 : Set Raising Interrupt
		EXTI->RTSR |= 1<<5;
		// 8 : Clear Falling Tigger
		EXTI->FTSR &= ~(1<<5);
		// 9 : Set NVIC Register
			// 9.1 : NVIC Interrupt Set Enable Regiter




	}

}

void GPIO_Toggle(GPIO_TypeDef *pGPIOx,uint8_t Pin)
{

	pGPIOx ->ODR ^= (1<<Pin);
}

uint8_t GPIO_Read(GPIO_TypeDef *pGPIOx, uint8_t Pin )
{
return ( (pGPIOx->IDR >> Pin) & 1);
}



void EXTI_IRQHandler( uint8_t IRQ, uint8_t ENorDI )
{
	if(EN)
	{
	// 1 : Set NVIC Register
		// 1.1 : NVIC Interrupt Set Enable Regiter
			*(NVIC_ISER0)	|= (1<<IRQ);
			//*(NVIC_ICER0)	|= (0<<IRQ);
	}else if(DI)
	{
		// 1.2 : NVIC INterrupt Clear Register
			*(NVIC_ICER0)	|= (1<<IRQ);

	}
}


void EXTI_IRQPriority(uint8_t IRQ,uint8_t Priority)
{

	// 1.3 : NVIC Priority Register
	uint32_t ipr			=	IRQ/4;
	uint8_t ipr_section = 	IRQ%4;
	uint8_t shift 		=	(ipr_section * 8) + (8 - NO_PR_BITS_IMPLEMENTED);
	uint32_t *NVIC_IPR_x = 	((NVIC_IPR + (ipr)));

	*NVIC_IPR_x	|= ( Priority << shift );
	//*(NVIC_IPR + (ipr*4)) |= (Priority << shift );
	//*NVIC_IPR_5	|= (Priority << shift );
}

void GPIO_IRQHandler(uint8_t Pin)
{

if( EXTI->PR & (1 << Pin) )
{
		EXTI->PR |= 1<<Pin;//clear pending bit

}
EXTI_IRQHandler(23, DI);


}



