#include "Timer_3_PIT.h"

extern struct Servo* servo_1;
extern struct Servo* servo_2;

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
  
	NVIC_EnableIRQ(TIM3_IRQn); //Register interrupt in NVIC
	TIM3->EGR |= TIM_EGR_UG;
	TIM3->CR1 |= TIM_CR1_URS;
	
	TIM3->DIER |= TIM_DIER_UIE;
	TIM3->CR1 |= TIM_CR1_CEN;
}



void TIM3_IRQHandler(void)
{
	if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF)
	{
		executeNextInstruction(servo_1);
		executeNextInstruction(servo_2);
	}
	TIM3->SR &= ~(TIM_SR_UIF);
}  

