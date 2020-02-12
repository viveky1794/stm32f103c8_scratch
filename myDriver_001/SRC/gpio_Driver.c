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

}

void GPIO_Toggle(GPIO_TypeDef *pGPIOx,uint8_t Pin)
{

	pGPIOx ->ODR ^= (1<<Pin);
}
