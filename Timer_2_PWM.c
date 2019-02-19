#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "Timer_2_PWM.h"




//that checks if you get a capture or an input capture or an overflow and depending on which one it is, it either turns on the red LED for the over flow or it time stamps
/* void TIM2_IRQHandler  (void) 
{ 
	if((TIM2->SR & TIM_SR_CC1IF) == TIM_SR_CC1IF)
	{
		time = timeUpdate(time);
		Green_LED_Toggle();
	}
	else if((TIM2->SR & TIM_SR_UIF) == TIM_SR_UIF)
	{
		Red_LED_On();
		TIM2->CR1 &= ~(TIM_CR1_CEN); //Turn off timer 2
		POST_FLAG = -1;
	}
	TIM2->SR &= ~(TIM_SR_CC1IF | TIM_SR_UIF);
} */
//it steps PA0 as an alternate function and that alternate function is timer2 channel 1
void Init_GPIO(void)
{
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOAEN; //enable periph clock for GPIO Port A
	
	
	GPIOA->MODER &= ~(0xFFFFFFFF); //PA0 TIM2 CH1 clear everything

	
	GPIOA->MODER |= 2U; //set PA0 to AF
	GPIOA->MODER |= 10<<2; //set PA1 AF
	
	GPIOA->AFR[0] &= ~(15U); //clear PA0 alternate function register
	GPIOA->AFR[0] &= ~(15U); //clear PA0 alternate function register
	
	GPIOA->AFR[0] |= (1U); //set PA0 to AF1 - Timer 2 Ch1 output
	GPIOA->AFR[0] |= 1<<4; //set PA1 to AF1 - Timer 2 Ch2 output
}



//boom, intialized timer 2 as PWM
void Init_Timer2()
{
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN; //Enable clock for Timer 2
	
	TIM2->CCER &= ~(TIM_CCER_CC1E); //disable capture compare to be able to write to control registers 
	TIM2->CR1 &= ~(TIM_CR1_CEN); //count enable
	
	TIM2->CCMR1 &= ~(TIM_CCMR1_CC1S); //Timer 2 is an output channel 1
	TIM2->CCMR1 &= ~(TIM_CCMR1_CC2S); //Timer 2 is an output channel 2
	
	TIM2->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk); //Timer 2 does not change level on Channel 1
	
		
	TIM2->ARR = 200; //20ms period
	TIM2->PSC = 8000U;
	

	//PWM Mode
	TIM2->CCMR1 &= ~(0x303); //1111: fSAMPLING=fDTS/32, N=8  .... 11: capture is done once every 8 events
	
	TIM2->CCMR1 |= TIM_CCMR1_OC1PE; //Preload register on TIMx_CCR1 enabled. Read/Write operations access the preload register. TIMx_CCR1 preload value is loaded in the active register at each update event.
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE; //Preload register on TIMx_CCR1 enabled. Read/Write operations access the preload register. TIMx_CCR1 preload value is loaded in the active register at each update event.
	
	TIM2->CCMR1 |= 0110U <<4; //0110: PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1 else inactive.
	TIM2->CCMR1 |= 0110U <<12; //0110: PWM mode 1 - In upcounting, channel 2 is active as long as TIMx_CNT<TIMx_CCR1 else inactive.
	
	TIM2->CR1   |= TIM_CR1_ARPE; //ARPE:Auto-reload pre load enable,1: TIMx_ARR register is buffered
	
	TIM2->CCER  |= TIM_CCER_CC1E; //1: On - OC1 signal is output on the corresponding output pin channel 1
	TIM2->CCER  |= TIM_CCER_CC2E; //1: On - OC1 signal is output on the corresponding output pin channel 1
	
	
	TIM2->ARR = 200; //auto relod with
	TIM2->PSC = 8000; //20ms period

	TIM2->CR1 = 0x1; //enable the count
}

//stops the timer
void Stop_Timer2(void)
{
	TIM2->CR1 &= ~TIM_CR1_CEN;
}
