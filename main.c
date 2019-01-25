#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "UART.h"


char RxComByte = 0;
uint8_t buffer[BufferSize];
char str[] = "The count of the timer is: \r\n";

int main(void)
{
	System_Clock_Init(); // Switch System Clock = 80 MHz
	LED_Init();
	UART2_Init();
	RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;	// enable clock for B group of GPIO
	GPIOB->MODER &= ~(0x3<<(2*6));										// set PB6 to alternate 
	GPIOB->MODER |= 0x02 << (2*6);
	GPIOB->AFR[0] |= 0x2 << (4*6);  // choose TIMER4 CH1 to PB6
	
	//400KHz (00) 2MHz (01) 10MHz (10) 40MHz (11)
	GPIOB->OSPEEDR &= ~(0x03<<(2*6)); //speed mask
	GPIOB->OSPEEDR |= 0x03 << (2*6);
	GPIOB->PUPDR |= (0x00<<(2*6));
	
	GPIOB->OTYPER &= ~(1<<6);
	
	RCC->AHB1ENR |= RCC_APB1ENR1_TIM4EN; //enable clock of timer 2
	TIM4->PSC = 2097000/1000 -1;		//timer frequency of 1000Hz
	
	TIM4->ARR 	= 1000;		//Auto-reload value
	TIM4->CCR1 	= 500;		//Compare and output register
	
	//Set OC1M of channel 1 to 011: OC1REF toggles when CNT matches CCR1
	TIM4->CCMR1 = TIM_CCMR1_OC1M_0 | TIM_CCMR1_OC1M_1;
	
	//IC1PSC bits (input capture 1 prescaler)
	//TIM4->CCMR1 &= ~TIM_CCER_CC1E;
	
	//Enable related interrupts
	//TIM4->DIER	|= TIM_DIER_CC1IE;
	
	//Enaable compare output
	TIM4->CCER = TIM_CCER_CC1E;
	//Enable the counter
	TIM4->CR1 |= TIM_CR1_CEN;
	
	//Set priority to 1
	//NVIC_SetPriority(TIM4_IRQn, 1);
	
	//NVIC_EnableIRQ(TIM4_IRQn);
	//__enable_irq();
	
	
	char rxByte;
	int count;
	int n;
	
	
	while(1)
	{
		count = TIM4->CNT;
		n = sprintf((char *)buffer, "count = %d\t", count);
		USART_Write(USART2, (uint8_t *)str, strlen(str));	
		USART_Write(USART2, buffer, n);  
		
		//Read GPIO input and then test for rising edge	
		//time the rising edge 100ms is the max count for the timer 
		//Interrupt on overflow
	}
}


