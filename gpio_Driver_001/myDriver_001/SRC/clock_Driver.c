/*
 * clock_Driver.c
 *
 *  Created on: Feb 12, 2020
 *      Author: vivek
 */

#include "clock_Driver.h"

void set_RCC_HSI_Clock_AHB_xx()
{
	// 1: set HSI as clock
	RCC->CR  |= RCC_CR_HSION;
	// 2: wait for HSI to be ready
	while(!(RCC->CR & RCC_CR_HSIRDY));
	// 3: select HSI as system Clock
	RCC->CFGR |= RCC_CFGR_SW_HSI;
	// 4: wait for HSI to be ready

}


void set_RCC_HSI_Clock_AHB_40MHz()
{
	// 1: #HSI oscillator clock / 2 selected as PLL input clock when PLL is disable
	RCC->CFGR |= HSI_DIV_2;
	// 2:
	RCC->CFGR |= PLLMULL_x_10;// setting PLL for 40 MHz


	// 3: set HSI as clock
	RCC->CR  |= RCC_CR_HSION;
	// 4: wait for HSI to be ready
	while(!(RCC->CR & RCC_CR_HSIRDY));


	// 5: Enable PLL
	RCC->CR |= PLL_ON;
	// 6: check PLL ready Flag
	while(!(RCC->CR & PLL_RDY_FLAG ));

	// 7: select PLL as system Clock
	RCC->CFGR |= 0x00000002;
	// 8: wait for PLL to be ready
	while( !( RCC->CFGR & 0x00000008 ) );

	// 9: AHB preScaler div by 1
	RCC->CFGR |= 0x0 << 7;


}

void set_RCC_HSI_Clock_APB2_1MHz()
{

	// 1: #HSI oscillator clock / 2 selected as PLL input clock when PLL is disable
		RCC->CFGR |= HSI_DIV_2;
		// 2:
		RCC->CFGR |= PLLMULL_x_4;// setting PLL for 16 MHz
		// 3: APB2 presclaer AHB Div by 2 means 16/16 = 1Mhz
		RCC->CFGR |= 0x7<<11 ;


		// 4: set HSI as clock
		RCC->CR  |= RCC_CR_HSION;
		// 5: wait for HSI to be ready
		while(!(RCC->CR & RCC_CR_HSIRDY));


		// 6: Enable PLL
		RCC->CR |= PLL_ON;
		// 7: check PLL ready Flag
		while(!(RCC->CR & PLL_RDY_FLAG ));

		// 8: select PLL as system Clock
		RCC->CFGR |= 0x00000002;
		// 9: wait for PLL to be ready
		while( !( RCC->CFGR & 0x00000008 ) );

		// 10: AHB preScaler div by 1
		RCC->CFGR |= 0x0 << 7;

}



void TIMER1()
{
	// 1: Enable Timer 1 APB2 BUS
	RCC->APB2ENR |= 1<<11;



}


