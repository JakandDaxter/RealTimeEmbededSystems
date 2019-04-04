#include "Timer_3_PIT.h"
#include <stdlib.h>

void Timer3_Init(void)
{	
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; //Enable clock for Timer 3
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOCEN; //Enable GPIO for Timer 3
	
	GPIOC->MODER &= ~(1U<<6); //PC6 set to alternate function mode
	GPIOC->AFR[0] |= (1U<<6); //PC6 set to TIM3 Channel 1
	
	TIM3->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E); //disable capture compare to be able to write to control registers 
	TIM3->CR1 &= ~(TIM_CR1_CEN);

	TIM3->ARR = PERIOD;
	TIM3->PSC = 8000U;
  
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_URS;
	
	TIM3->DIER |= TIM_DIER_UIE;
	TIM3->CR1 |= TIM_CR1_CEN;
}

