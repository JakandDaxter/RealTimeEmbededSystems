#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "UART.h"
#include "LED.h"
#include "Timer_2_PWM.h"


uint8_t buffer[BufferSize];

int main(void)
{
	uint8_t buffer[BufferSize];
	__disable_irq();// disables interrupts
	System_Clock_Init(); 
	LED_Init();
	UART2_Init();
	Init_Timer2();
	__enable_irq();
	
	USART_Write(USART2, buffer, );
	
}
