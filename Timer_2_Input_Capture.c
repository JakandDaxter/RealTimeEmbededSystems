#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "Timer_2_Input_Capture.h"


uint32_t delta_time[ARRAY_SIZE];
static int i = 0;
int POST_FLAG = 0;
//
struct time_stamp
{
	uint32_t stamp[2];
	uint8_t i;
};

struct time_stamp time;

//that is a function in what is does is that, once it gets tow time stamps, does this math and then stores it into delta time
struct time_stamp timeUpdate(struct time_stamp now)
{
	now.stamp[now.i] = TIM2->CCR1;
	if((now.i == 1) && (i < ARRAY_SIZE)) 
	{
		delta_time[i] = (now.stamp[1] - now.stamp[0])/80U; //80MHz / (t2 - t1) = freq of rising edges
		i++;
		TIM2->CNT = 0;
	}
	now.i ^= 1;
	return now;
}
//returns time at that index
uint32_t get_delta_time(int i)
{
	return delta_time[i];
}

//that checks if you get a capture or an input capture or an overflow and depending on which one it is, it either turns on the red LED for the over flow or it time stamps
void TIM2_IRQHandler  (void) 
{ 
	if((TIM2->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF)
	{
		time = timeUpdate(time);
		POST_FLAG = 1;
	}
	else if((TIM2->SR & TIM_SR_UIF) == TIM_SR_UIF)
	{
		Red_LED_On();
		TIM2->CR1 &= ~(TIM_CR1_CEN); //Turn off timer 2
	}
	TIM2->SR &= ~(TIM_SR_CC1IF | TIM_SR_UIF);
	POST_FLAG = -1;
}
//it steps PA0 as an alternate function and that alternate function is timer2 channel 1
void Init_GPIO(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //enable periph clock for GPIO Port A
	
	
	GPIOA->MODER &= ~3U; //clear PA0
	GPIOA->MODER |= 2U; //set PA0 to AF
	
	GPIOA->AFR[0] &= ~(15U); //clear PA0 alternate function register
	GPIOA->AFR[0] |= (1U); //set PA0 to AF1 - Timer 2 Ch1 output
}

//
// Init_Timer2(uint32_t arr)
//
// Parameters:
// 	arr -> value TIM2 will count up to before resetting to 0
// 	TIM2CLK/ARR = Freq of TIM2
//
// Description:
// Configures TIM2 for input capture on channel 1 -> PA0 on STM32 Discovery Board
// Registers the ISR into NVIC and enables interrupt on capture

//boom, intialized timer 2
void Init_Timer2(uint32_t arr)
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //Enable clock for Timer 2
	LED_Init();
	
	TIM2->CCER &= ~(TIM_CCER_CC1E); //disable capture compare to be able to write to control registers 
	TIM2->ARR = arr; //Timer 2 auto reload register
	
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1PE | 3U | (1U<<2) | (1U<<3)); //Timer 2 No preload no fast enable
	TIM2->CCMR1 |= TIM_CCMR1_CC1S_0; // select Input Capture mode 
	TIM2->CCMR1 &= ~(TIM_CCER_CC1NP | TIM_CCER_CC1P); // Clears the two bits making it rising edge detector
	
	TIM2->CR1 |= TIM_CR1_URS; //Only overflow generates an update event 
	
	TIM2->DIER  |= (TIM_DIER_CC1IE | TIM_DIER_UIE); //enables interrupts
	TIM2->CCER |= TIM_CCER_CC1E; //enablt capture compare
	
	NVIC_EnableIRQ(TIM2_IRQn); //enable the interrupt
}

//clears the struct then starts the timer
void Start_Timer2(void)
{
	time.stamp[0] = 0;
	time.stamp[1] = 0;
	time.i = 0;
	TIM2->CNT = 0;
	TIM2->CR1 |= TIM_CR1_CEN;
}
//stops the timer
void Stop_Timer2(void)
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
}

