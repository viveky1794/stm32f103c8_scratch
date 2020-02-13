
#include "stm32f103C8_MCU.h"

#include "gpio_Driver.h"
#include "clock_Driver.h"

void delay()
{

for(uint32_t i=0;i<605000;i++);

}



/***********	IRQ Handler	-> Interrupt Service Routine	************/
void EXTI9_5_IRQHandler(void)
{

GPIO_IRQHandler(5);
delay();

}






int main(void)
{
	set_RCC_HSI_Clock_APB2_1MHz();

	GPIO_Init(GPIOC,GPIO_PIN_14,GPIO_Mode_IPU, GPIO_Speed_INPUT);
	GPIO_Init(GPIOC,GPIO_PIN_13,GPIO_Mode_Out_PP, GPIO_Speed_2MHz);
	GPIO_Init(GPIOB,GPIO_PIN_5,GPIO_Mode_AF_PP, GPIO_Speed_2MHz);
	
	EXTI_IRQHandler( 23, EN );
	EXTI_IRQPriority(23, 15);
	
	
	for(;;)
	{
			if( GPIO_Read(GPIOC,14) )
			{
				GPIOC->BSRR |= BSRR_BIT_SET(13);
				delay();
			}else
			{
				GPIOC->BRR |= BRR_BIT(13);
				
			}
	}


}
