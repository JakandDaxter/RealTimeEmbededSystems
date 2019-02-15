#include "Timer_5_PIT.h"
#include "States.h"
#include "Servo_CL_Compiler.h"

static uint8_t commands[19] = {MOV+0,MOV+5,MOV+0,MOV+3,LOOP+0,MOV+1,MOV+4,END_LOOP,MOV+0,MOV+2,WAIT+0,MOV+3,WAIT+0,MOV+2,MOV+3,WAIT+31,WAIT+31,WAIT+31,MOV+4};
static struct ServoCallStack* callStack;

void Timer5_Init(void)
{
	load_recipie_servo2();
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM5EN; //Enable clock for Timer 3
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOFEN; //Enable GPIO for Timer 3
	
	GPIOF->MODER &= ~(1U<<6); //PF6 set to alternate function mode
	GPIOF->AFR[0] |= (2U<<6); //PF6 set to TIM5 Channel 1
	
	TIM5->CCER &= ~(TIM_CCER_CC1E); //disable capture compare to be able to write to control registers 
	TIM5->CR1 &= ~(TIM_CR1_CEN);
	
	TIM5->CCMR1 &= ~(TIM_CCMR1_CC1S); //Timer 3 is an output
	TIM5->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk); //Timer 3 does not change level on Channel 1

	TIM5->ARR = PERIOD;
	TIM5->PSC = 8000U;
	
	
	NVIC_EnableIRQ(TIM5_IRQn); //Register interrupt in NVIC
	TIM5->EGR |= TIM_EGR_UG;
	TIM5->CR1 |= TIM_CR1_URS;
}

void load_recipie_servo2(void)
{
	callStack = createServoCallStack(commands,19);
}

void start_timer5(void)
{
	load_recipie_servo2();
	TIM5->DIER |= TIM_DIER_UIE;//Enable update interrupt for timer 3
	TIM5->CR1 |= TIM_CR1_CEN;
}

void restart_timer5(void)
{
	TIM5->DIER |= TIM_DIER_UIE;//Enable update interrupt for timer 3
	TIM5->CR1 |= TIM_CR1_CEN;
}

void stop_timer5(void)
{
	TIM5->DIER &= ~TIM_DIER_UIE;
	TIM5->CR1 &= ~TIM_CR1_CEN;
}

void TIM5_IRQHandler(void)
{
	if((TIM5->SR & TIM_SR_UIF) == TIM_SR_UIF)
	{
		Green_LED_Toggle();
		if(executeNextInstruction(callStack) == -1)
		{
			TIM5->CR1 &= ~TIM_CR1_CEN;
		}
		TIM5->CNT = 0;
	}
	TIM5->SR &= ~(TIM_SR_UIF);
} 