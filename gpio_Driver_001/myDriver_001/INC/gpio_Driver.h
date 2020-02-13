/*
 * gpio_Driver.h
 *
 *  Created on: Feb 12, 2020
 *      Author: vivek
 */

#ifndef INC_GPIO_DRIVER_H_
#define INC_GPIO_DRIVER_H_

#include "stm32f103C8_MCU.h"

#define AFIO_EN	(1<<0)
#define IOPAEN	(1<<2)
#define IOPBEN	(1<<3)
#define IOPCEN	(1<<4)
#define Enable_RCC_Pheririal_BUS(port) 		do{	RCC->APB2ENR |= port;\
										}while(0);



#define	ODR_BIT_SET(x)		(1<<x)
#define	ODR_BIT_RESET(x)	(0<<x)

#define BSRR_BIT_SET(x)		(1<<x)
#define BSRR_BIT_RESET(x)	(1<<(x+16))
#define BRR_BIT(x)			(1<<x)


typedef enum
{
  GPIO_Speed_INPUT = 0,
  GPIO_Speed_10MHz = 1,
  GPIO_Speed_2MHz,
  GPIO_Speed_50MHz
}GPIOSpeed_TypeDef;


typedef enum
{ GPIO_Mode_AIN = 0x0,
  GPIO_Mode_IN_FLOATING = 0x04,
  GPIO_Mode_IPD = 0x28,
  GPIO_Mode_IPU = 0x8,
  GPIO_Mode_Out_OD = 01,
  GPIO_Mode_Out_PP = 00,
  GPIO_Mode_AF_OD = 02,
  GPIO_Mode_AF_PP = 03
}GPIOMode_TypeDef;

typedef enum
{
	GPIO_PIN_1 = 1,
	GPIO_PIN_2,
	GPIO_PIN_3,
	GPIO_PIN_4,
	GPIO_PIN_5,
	GPIO_PIN_6,
	GPIO_PIN_7,
	GPIO_PIN_8,
	GPIO_PIN_9,
	GPIO_PIN_10,
	GPIO_PIN_11,
	GPIO_PIN_12,
	GPIO_PIN_13,
	GPIO_PIN_14,
	GPIO_PIN_15,
}GPIO_PIN;



void GPIO_Init(		GPIO_TypeDef *pGPIOx,
					uint8_t Pin,
					uint8_t Mode,
					uint8_t Speed		);

void GPIO_Toggle(GPIO_TypeDef *pGPIOx,uint8_t Pin);

uint8_t GPIO_Read(GPIO_TypeDef *pGPIOx, uint8_t Pin );

void EXTI_IRQHandler( uint8_t IRQ, uint8_t ENorDI );
void EXTI_IRQPriority(uint8_t IRQ,uint8_t Priority);
void GPIO_IRQHandler(uint8_t Pin);


#endif /* INC_GPIO_DRIVER_H_ */
