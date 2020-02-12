/*
 * clock_Driver.c
 *
 *  Created on: Feb 12, 2020
 *      Author: vivek
 */

#include "clock_Driver.h"

void set_RCC_HSI_Clock()
{
	// 1: set HSI as clock
	RCC->CR  |= RCC_CR_HSION;
	// 2: wait for HSI to be ready
	while(!(RCC->CR & RCC_CR_HSIRDY));
	// 3: select HSI as system Clock
	RCC->CFGR |= RCC_CFGR_SW_HSI;
	// 4: wait for HSI to be ready

}

