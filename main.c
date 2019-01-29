#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "UART.h"

uint8_t buffer[BufferSize];

void Init_GPIO(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //enable periph clock for GPIO Port A
	
	GPIOA->MODER &= ~3U; //clear PA0
	GPIOA->MODER |= 2U; //set PA0 to AF
	
	GPIOA->AFR[0] &= ~(15U); //clear PA0 alternate function register
	GPIOA->AFR[0] |= (1U); //set PA0 to AF1 - Timer 2 Ch1 output
}

void Init_Timer2(void)
{
	//1. Select the counter clock (internal, external, prescaler).
	//2. Write the desired data in the TIMx_ARR and TIMx_CCRx registers.
	//3. Set the CCxIE and/or CCxDE bits if an interrupt and/or a DMA request is to be
	//generated.
	//4. Select the output mode. For example, you must write OCxM=011, OCxPE=0, CCxP=0
	//and CCxE=1 to toggle OCx output pin when CNT matches CCRx, CCRx preload is not
	//used, OCx is enabled and active high.
	//5. Enable the counter by setting the CEN bit in the TIMx_CR1 register.
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //Enable clock for Timer 2
	
	TIM2->CCER &= ~(TIM_CCER_CC1E);
	TIM2->ARR = 40000U; //Timer 2 auto reload register
	TIM2->CCR1 = 20000U; //Timer 2 capture compare 
	
	//TIM2->CCMR1 |= TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1; //Timer 2 OC1M=011 -> Toggle on compare
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1PE | 3U | (1U<<2) | (1U<<3)); //Timer 2 No preload no fast enable
	TIM2->CCER |= TIM_CCER_CC1E;
	TIM2->EGR = TIM_EGR_UG;
	//and channel 1 is an output
}

void Start_Timer2()
{
	TIM2->CR1 |= TIM_CR1_CEN;
}

int main(void)
{
	System_Clock_Init();
	Init_GPIO();
	Init_Timer2();
	UART2_Init();
	Start_Timer2();
	
	int n;
	while(1)
	{
		
		n = sprintf((char *)buffer, "a = %d\t", TIM2->CNT);
		USART_Write(USART2, buffer, n);		
		// now spin for a while to slow it down
		//for (uint32_t i = 0; i < 4000000; i++)
			//;
	}
}


