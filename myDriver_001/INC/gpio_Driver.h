/*
 * gpio_Driver.h
 *
 *  Created on: Feb 12, 2020
 *      Author: vivek
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stm32f103C8_MCU.h"

#define IOPAEN	(1<<2)
#define IOPBEN	(1<<3)
#define IOPCEN	(1<<4)
#define Enable_RCC_Pheririal_BUS(port) 		do{	RCC->APB2ENR |= port;\
										}while(0);

typedef enum
{
  GPIO_Speed_10MHz = 1,
  GPIO_Speed_2MHz,
  GPIO_Speed_50MHz
}GPIOSpeed_TypeDef;


typedef enum
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x48,
  GPIO_Mode_Out_OD = 01,
  GPIO_Mode_Out_PP = 00,
  GPIO_Mode_AF_OD = 02,
  GPIO_Mode_AF_PP = 03
}GPIOMode_TypeDef;


typedef struct
{

}GPIO_Handler;


void GPIO_Init(		GPIO_TypeDef *pGPIOx,
					uint8_t Pin,
					uint8_t Mode,
					uint8_t Speed		);

void GPIO_Toggle(GPIO_TypeDef *pGPIOx,uint8_t Pin);

#endif /* INC_GPIO_DRIVER_H_ */
