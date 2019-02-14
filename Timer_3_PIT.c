#include "Timer_3_PIT.h"

void Timer3_Init(void)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; //Enable clock for Timer 3
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN; //Enable GPIO for Timer 3
	
	GPIOE->MODER &= ~(1U<<3); //PE3 set to alternate function mode
	GPIOE->AFR[0] |= (1U<<3); //PE3 set to TIM3 Channel 1
	
	
	TIM3->CCER &= ~(TIM_CCER_CC1E); //disable capture compare to be able to write to control registers 
	TIM3->CR1 &= ~(TIM_CR1_CEN);
	
	TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S); //Timer 3 is an output
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk); //Timer 3 does not change level on Channel 1

	TIM3->ARR = PERIOD;
	TIM3->PSC = 40000U;
	
	TIM3->DIER |= TIM_DIER_UIE; //Enable update interrupt for timer 3
	
	NVIC_EnableIRQ(TIM3_IRQn); //Register interrupt in NVIC
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_URS;
	TIM3->CR1 |= TIM_CR1_CEN; //start timer
}

void TIM3_IRQHandler(void)
{
	if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF)
	{
		Red_LED_Toggle();
		TIM3->CNT = 0;
	}
	TIM3->SR &= ~(TIM_SR_UIF);
}  

