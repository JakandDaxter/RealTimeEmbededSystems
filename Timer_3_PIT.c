#include "Timer_3_PIT.h"
#include "States.h"
#include "Servo_CL_Compiler.h"

static uint8_t commands[19] = {MOV+5,MOV+0,MOV+5,MOV+3,LOOP+0,MOV+1,MOV+4,END_LOOP,MOV+0,MOV+2,WAIT+0,MOV+3,WAIT+0,MOV+2,MOV+3,WAIT+31,WAIT+31,WAIT+31,MOV+5};
static struct ServoCallStack* callStack;

void Timer3_Init(void)
{
	load_recipie();
	
	RCC->APB1ENR1 |= RCC_APB1ENR1_TIM3EN; //Enable clock for Timer 3
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOEEN; //Enable GPIO for Timer 3
	
	GPIOE->MODER &= ~(1U<<3); //PE3 set to alternate function mode
	GPIOE->AFR[0] |= (1U<<3); //PE3 set to TIM3 Channel 1
	
	
	TIM3->CCER &= ~(TIM_CCER_CC1E); //disable capture compare to be able to write to control registers 
	TIM3->CR1 &= ~(TIM_CR1_CEN); //count enable
	
	TIM3->CCMR1 &= ~(TIM_CCMR1_CC1S); //Timer 3 is an output
	TIM3->CCMR1 &= ~(TIM_CCMR1_OC1M_Msk); //Timer 3 does not change level on Channel 1
		
	TIM3->ARR = PERIOD;
	TIM3->PSC = 8000U;
	

	//PWM Mode
	TIM3->CCMR1 &= ~(0x303); //1111: fSAMPLING=fDTS/32, N=8  .... 11: capture is done once every 8 events
	TIM3->CCMR1 |= TIM_CCMR1_OC1PE; //Preload register on TIMx_CCR1 enabled. Read/Write operations access the preload register. TIMx_CCR1 preload value is loaded in the active register at each update event.
	TIM3->CCMR1 |= 0110U <<4; //0110: PWM mode 1 - In upcounting, channel 1 is active as long as TIMx_CNT<TIMx_CCR1 else inactive.
	TIM3->CR1   |= TIM_CR1_ARPE; //ARPE:Auto-reloadpreloadenable,1: TIMx_ARR register is buffered
	TIM3->CCER  |= TIM_CCER_CC1E; //1: On - OC1 signal is output on the corresponding output pin
	
	//TIM3->ARR = 200; //auto relod with
	//TIM3->PSC = 8000; //20ms period
	TIM3->SR &= ~TIM_SR_UIF; // clear update flag
	//tim3->CCMR1 |=TIM_CCMR1_ 
	//---------------------------------------------
	TIM3->DIER |= TIM_DIER_UIE; //Enable update interrupt for timer 3
	
	NVIC_EnableIRQ(TIM3_IRQn); //Register interrupt in NVIC
	TIM3->EGR |= TIM_EGR_UG;//load prescaler
	TIM3->CR1 |= TIM_CR1_URS;
}

void load_recipie(void)
{
	callStack = createServoCallStack(commands,19);
}

void start_timer3(void)
{
	load_recipie();
	TIM3->DIER |= TIM_DIER_UIE;//Enable update interrupt for timer 3
	TIM3->CR1 |= TIM_CR1_CEN;
}

void restart_timer3(void)
{
	TIM3->DIER |= TIM_DIER_UIE;//Enable update interrupt for timer 3
	TIM3->CR1 |= TIM_CR1_CEN;
}

void stop_timer3(void)
{
	TIM3->DIER &= ~TIM_DIER_UIE;
	TIM3->CR1 &= ~TIM_CR1_CEN;
}

void TIM3_IRQHandler(void)
{
	if((TIM3->SR & TIM_SR_UIF) == TIM_SR_UIF)
	{
		Red_LED_Toggle();
		if(executeNextInstruction(callStack) == -1)
		{
			TIM3->CR1 &= ~TIM_CR1_CEN;
		}
		TIM3->CNT = 0;
	}
	TIM3->SR &= ~(TIM_SR_UIF);
}  

//function to adjust duty cycle

void Dootypotootie(uint8_t pwmNum, uint32_t pwmLimit){
	
	
}


