#include "Timer_3_PIT.h"
#include "States.h"
#include "Servo_CL_Compiler.h"
#include "Servo.h"
#include <stdlib.h>

static uint8_t commands[20] = {MOV+0,MOV+5,MOV+0,MOV+3,LOOP+0,MOV+1,MOV+4,END_LOOP,MOV+0,MOV+2,WAIT+0,MOV+3,WAIT+0,MOV+2,MOV+3,WAIT+31,WAIT+31,WAIT+31,MOV+0,0};
static uint8_t commands2[19] = {MOV+5,MOV+0,MOV+5,MOV+3,LOOP+0,MOV+1,MOV+4,END_LOOP,MOV+0,MOV+2,WAIT+0,MOV+3,WAIT+0,MOV+2,MOV+3,WAIT+31,WAIT+31,WAIT+31,MOV+5};
struct Servo* servo_1;
struct Servo* servo_2;


void Timer3_Init(void)
{	
	load_recipies();
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

void load_recipies()
{
	servo_1 = createServo(commands, 20);
	servo_1->id = 1;
	servo_2 = createServo(commands2, 19);
	servo_2->id = 2;
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

