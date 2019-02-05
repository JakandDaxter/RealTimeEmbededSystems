#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"

uint32_t delta_time[1000];
int i = 0;

struct time_stamp
{
	uint32_t stamp[2];
	uint8_t i;
};

struct time_stamp time;

struct time_stamp timeUpdate(struct time_stamp now)
{
	now.stamp[now.i] = TIM2->CCR1;
	if(now.i == 1)
	{
		delta_time[i] = 0x04C46400/(now.stamp[1] - now.stamp[0]); //80MHz / (t2 - t1) = freq of rising edges
		i++;
		TIM2->CNT = 0;
	}
	now.i ^= 1;
	return now;
}

uint32_t get_delta_time(int i)
{
	return delta_time[i];
}


void TIM2_IRQHandler  (void) 
{ 
	if((TIM2->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF)
	{
		time = timeUpdate(time);
	}
	TIM2->SR &= ~(TIM_SR_CC1IF);
}

void Init_GPIO(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //enable periph clock for GPIO Port A
	
	GPIOA->MODER &= ~3U; //clear PA0
	GPIOA->MODER |= 2U; //set PA0 to AF
	
	GPIOA->AFR[0] &= ~(15U); //clear PA0 alternate function register
	GPIOA->AFR[0] |= (1U); //set PA0 to AF1 - Timer 2 Ch1 output
}

/*
Init_Timer2(uint32_t arr)

Parameters:
	arr -> value TIM2 will count up to before resetting to 0 
	TIM2CLK/ARR = Freq of TIM2

Description: 
Configures TIM2 for input capture on channel 1 -> PA0 on STM32 Discovery Board
Registers the ISR into NVIC and enables interrupt on capture
*/
void Init_Timer2(uint32_t arr)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //Enable clock for Timer 2
	
	TIM2->CCER &= ~(TIM_CCER_CC1E); //disable capture compare to be able to write to control registers 
	TIM2->ARR = arr; //Timer 2 auto reload register
	
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1PE | 3U | (1U<<2) | (1U<<3)); //Timer 2 No preload no fast enable
	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0; // select Input Capture mode 
	TIM2->CCMR1 &= ~(TIM_CCER_CC1NP | TIM_CCER_CC1P); // Clears the two bits making it rising edge detector
	
	TIM2->DIER  |= TIM_DIER_CC1IE; //enables interrupts
	
	TIM2->CCER |= TIM_CCER_CC1E; //enablt capture compare
	
	TIM2->EGR = TIM_EGR_UG; //sends update event to TIM2
	NVIC_EnableIRQ(TIM2_IRQn); //enable the interrupt
}

void Start_Timer2()
{
	time.stamp[0] = 0;
	time.stamp[1] = 0;
	time.i = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
}

