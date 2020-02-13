/*
 * stm32f103c8_MCU.h
 *
 *  Created on: Feb 12, 2020
 *      Author: vivek
 */

#ifndef INC_STM32F103C8_MCU_H_
#define INC_STM32F103C8_MCU_H_

#include "stdint.h"

#define __IO	volatile
#define __vo	__IO
#define EN	1
#define DI	0
//*************************************************************************************************//
//								EXIT INTREEUPT SECTION
//
//*************************************************************************************************//

/******************************	NVIC Register Address	******************************************
 *
 * 				Processor Specific	Interrupt Set-Enable Registers
 */

#define NVIC_ISER0				( (__vo uint32_t *)0xE000E100 )
#define NVIC_ISER1				( (__vo uint32_t *)0xE000E104 )
#define NVIC_ISER2				( (__vo uint32_t *)0xE000E108 )
#define NVIC_ISER3				( (__vo uint32_t *)0xE000E10C )
#define NVIC_ISER4				( (__vo uint32_t *)0xE000E110 )
#define NVIC_ISER5				( (__vo uint32_t *)0xE000E114 )
#define NVIC_ISER6				( (__vo uint32_t *(0xE000E118 )
#define NVIC_ISER7				( (__vo uint32_t *)0xE000E11C )

/*
 * 				Processor Specific	Interrupt Clear-Enable Registers
 */

#define NVIC_ICER0				( (__vo uint32_t*)(0XE000E180) )
#define NVIC_ICER1				( (__vo uint32_t*)(0XE000E184) )
#define NVIC_ICER2				( (__vo uint32_t*)(0XE000E188) )
#define NVIC_ICER3				( (__vo uint32_t*)(0XE000E18C) )
#define NVIC_ICER4				( (__vo uint32_t*)(0XE000E190) )
#define NVIC_ICER5				( (__vo uint32_t*)(0XE000E194) )
#define NVIC_ICER6				( (__vo uint32_t*)(0XE000E198) )
#define NVIC_ICER7				( (__vo uint32_t*)(0XE000E19C) )

/*
 * 			Processor Specific	Interrupt Set-Pending Registers
 */

#define NVIC_ISPR0				((__vo uint32_t *)0XE000E200)
#define NVIC_ISPR1				((__vo uint32_t *)0XE000E204)
#define NVIC_ISPR2				((__vo uint32_t *)0XE000E208)
#define NVIC_ISPR3				((__vo uint32_t *)0XE000E20C)
#define NVIC_ISPR4				((__vo uint32_t *)0XE000E210)
#define NVIC_ISPR5				((__vo uint32_t *)0XE000E214)
#define NVIC_ISPR6				((__vo uint32_t *)0XE000E218)
#define NVIC_ISPR7				((__vo uint32_t *)0XE000E21C)

/*
 * 			Processor Specific	Interrupt CLEAR-Pending Registers
 */

#define NVIC_ICPR0				(__vo uint32_t *)(0XE000E280)
#define NVIC_ICPR1				(__vo uint32_t *)(0XE000E284)
#define NVIC_ICPR2				(__vo uint32_t *)(0XE000E288)
#define NVIC_ICPR3				(__vo uint32_t *)(0XE000E28C)
#define NVIC_ICPR4				(__vo uint32_t *)(0XE000E290)
#define NVIC_ICPR5				(__vo uint32_t *)(0XE000E294)
#define NVIC_ICPR6				(__vo uint32_t *)(0XE000E298)
#define NVIC_ICPR7				(__vo uint32_t *)(0XE000E29C)

/*
 *			Processor Specific	Interrupt PRIORITY Registers
 */
#define NO_PR_BITS_IMPLEMENTED	4
#define NVIC_IPR_BaseAddr		( (__vo uint32_t *)(0xE000E400) )

typedef struct
{
	__IO uint32_t NVIC_IPR[60];

}NVIC_IPR_TypeDef;


#define NVIC_IPR				( (NVIC_IPR_BaseAddr) )
#define NVIC_IPR_5				( (__vo uint32_t *)(0xE000E414) )
/*
#define NVIC_IPR0				(__vo uint32_t *)(0xE000E400)
#define NVIC_IPR1				(__vo uint32_t *)(0xE000E404)
#define NVIC_IPR2				(__vo uint32_t *)(0xE000E408)
#define NVIC_IPR3				(__vo uint32_t *)(0xE000E40C)
#define NVIC_IPR4				(__vo uint32_t *)(0xE000E410)
#define NVIC_IPR5				(__vo uint32_t *)(0xE000E414)
#define NVIC_IPR6				(__vo uint32_t *)(0xE000E418)
#define NVIC_IPR7				(__vo uint32_t *)(0xE000E41C)
*/
// TODO : Priority Register is upto 59





/**
  * @brief Reset and Clock Control
  */

typedef struct
{
  __IO uint32_t CR;
  __IO uint32_t CFGR;
  __IO uint32_t CIR;
  __IO uint32_t APB2RSTR;
  __IO uint32_t APB1RSTR;
  __IO uint32_t AHBENR;
  __IO uint32_t APB2ENR;
  __IO uint32_t APB1ENR;
  __IO uint32_t BDCR;
  __IO uint32_t CSR;
  __IO  uint32_t AHBSTR;
  __IO uint32_t CFGR2;

} RCC_TypeDef;


/**
  * @brief General Purpose I/O
  */

typedef struct
{
  __IO uint32_t CRL;
  __IO uint32_t CRH;
  __IO uint32_t IDR;
  __IO uint32_t ODR;
  __IO uint32_t BSRR;
  __IO uint32_t BRR;
  __IO uint32_t LCKR;
} GPIO_TypeDef;

/**
  * @brief Alternate Function I/O
  */

typedef struct
{
  __IO uint32_t EVCR;
  __IO uint32_t MAPR;
  __IO uint32_t EXTICR[4];
  uint32_t RESERVED0;
  __IO uint32_t MAPR2;
} AFIO_TypeDef;


typedef struct
{
	__IO uint32_t IMR;
	__IO uint32_t EMR;
	__IO uint32_t RTSR;
	__IO uint32_t FTSR;
	__IO uint32_t SWIER;
	__IO uint32_t PR;
}EXTI_TypeDef;


/*!< Peripheral memory map */
#define PERIPH_BASE           ((uint32_t)0x40000000) /*!< Peripheral base address in the alias region */
#define APB1PERIPH_BASE       PERIPH_BASE
#define APB2PERIPH_BASE       (PERIPH_BASE + 0x10000)
#define AHBPERIPH_BASE        (PERIPH_BASE + 0x20000)

#define RCC_BASE              (AHBPERIPH_BASE + 0x1000)
#define AFIO_BASE             (APB2PERIPH_BASE + 0x0000)
#define EXTI_BASE             (APB2PERIPH_BASE + 0x0400)
#define GPIOA_BASE            (APB2PERIPH_BASE + 0x0800)
#define GPIOB_BASE            (APB2PERIPH_BASE + 0x0C00)
#define GPIOC_BASE            (APB2PERIPH_BASE + 0x1000)
#define GPIOD_BASE            (APB2PERIPH_BASE + 0x1400)
#define GPIOE_BASE            (APB2PERIPH_BASE + 0x1800)
#define GPIOF_BASE            (APB2PERIPH_BASE + 0x1C00)
#define GPIOG_BASE            (APB2PERIPH_BASE + 0x2000)

#define RCC                 ((RCC_TypeDef *) RCC_BASE)
#define AFIO                ((AFIO_TypeDef *) AFIO_BASE)
#define EXTI                ((EXTI_TypeDef *) EXTI_BASE)
#define GPIOA               ((GPIO_TypeDef *) GPIOA_BASE)
#define GPIOB               ((GPIO_TypeDef *) GPIOB_BASE)
#define GPIOC               ((GPIO_TypeDef *) GPIOC_BASE)
#define GPIOD               ((GPIO_TypeDef *) GPIOD_BASE)
#define GPIOE               ((GPIO_TypeDef *) GPIOE_BASE)
#define GPIOF               ((GPIO_TypeDef *) GPIOF_BASE)
#define GPIOG               ((GPIO_TypeDef *) GPIOG_BASE)



/******************************************************************************/
/*                                                                            */
/*                         Reset and Clock Control                            */
/*                                                                            */
/******************************************************************************/

/********************  Bit definition for RCC_CR register  ********************/
#define  RCC_CR_HSION                        ((uint32_t)0x00000001)        /*!< Internal High Speed clock enable */
#define  RCC_CR_HSIRDY                       ((uint32_t)0x00000002)        /*!< Internal High Speed clock ready flag */
#define  RCC_CR_HSITRIM                      ((uint32_t)0x000000F8)        /*!< Internal High Speed clock trimming */
#define  RCC_CR_HSICAL                       ((uint32_t)0x0000FF00)        /*!< Internal High Speed clock Calibration */
#define  RCC_CR_HSEON                        ((uint32_t)0x00010000)        /*!< External High Speed clock enable */
#define  RCC_CR_HSERDY                       ((uint32_t)0x00020000)        /*!< External High Speed clock ready flag */
#define  RCC_CR_HSEBYP                       ((uint32_t)0x00040000)        /*!< External High Speed clock Bypass */
#define  RCC_CR_CSSON                        ((uint32_t)0x00080000)        /*!< Clock Security System enable */
#define  RCC_CR_PLLON                        ((uint32_t)0x01000000)        /*!< PLL enable */
#define  RCC_CR_PLLRDY                       ((uint32_t)0x02000000)        /*!< PLL clock ready flag */


/*******************  Bit definition for RCC_CFGR register  *******************/
/*!< SW configuration */
#define  RCC_CFGR_SW                         ((uint32_t)0x00000003)        /*!< SW[1:0] bits (System clock Switch) */
#define  RCC_CFGR_SW_0                       ((uint32_t)0x00000001)        /*!< Bit 0 */
#define  RCC_CFGR_SW_1                       ((uint32_t)0x00000002)        /*!< Bit 1 */

#define  RCC_CFGR_SW_HSI                     ((uint32_t)0x00000000)        /*!< HSI selected as system clock */
#define  RCC_CFGR_SW_HSE                     ((uint32_t)0x00000001)        /*!< HSE selected as system clock */
#define  RCC_CFGR_SW_PLL                     ((uint32_t)0x00000002)        /*!< PLL selected as system clock */

/*!< SWS configuration */
#define  RCC_CFGR_SWS                        ((uint32_t)0x0000000C)        /*!< SWS[1:0] bits (System Clock Switch Status) */
#define  RCC_CFGR_SWS_0                      ((uint32_t)0x00000004)        /*!< Bit 0 */
#define  RCC_CFGR_SWS_1                      ((uint32_t)0x00000008)        /*!< Bit 1 */

#define  RCC_CFGR_SWS_HSI                    ((uint32_t)0x00000000)        /*!< HSI oscillator used as system clock */
#define  RCC_CFGR_SWS_HSE                    ((uint32_t)0x00000004)        /*!< HSE oscillator used as system clock */
#define  RCC_CFGR_SWS_PLL                    ((uint32_t)0x00000008)        /*!< PLL used as system clock */


#endif /* INC_STM32F103C8_MCU_H_ */
