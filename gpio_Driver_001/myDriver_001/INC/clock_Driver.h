/*
 * clock_Driver.h
 *
 *  Created on: Feb 12, 2020
 *      Author: vivek
 */

#ifndef INC_CLOCK_DRIVER_H_
#define INC_CLOCK_DRIVER_H_




#include "stm32f103C8_MCU.h"

#define PLL_ON				(1<<24)
#define PLL_RDY_FLAG		(1<<25)
#define HSI_DIV_2			(0<<16)
#define PLLMULL_x_10		(0x8 << 18)
#define PLLMULL_x_4			(0x2 << 18)

void set_RCC_HSI_Clock_AHB_xx();
void set_RCC_HSI_Clock_AHB_40MHz();
void set_RCC_HSI_Clock_APB2_1MHz();

#endif /* INC_CLOCK_DRIVER_H_ */
