#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "UART.h"
#include "LED.h"
#include "Timer_2_Input_Capture.h"

uint8_t buffer[BufferSize];

int main(void)
{
	__disable_irq();
	System_Clock_Init();
	Init_GPIO();
	LED_Init();
	Init_Timer2(0x7A1200);
	UART2_Init();
	__enable_irq();
	Start_Timer2();

	
	int n;
	while(1)
	{
		n = sprintf((char *)buffer, "a = %d\t", get_delta_time(0));
		USART_Write(USART2, buffer, n);
		//n = sprintf((char *)buffer, "b = %d\n\r", timeStamp[1]);
		//USART_Write(USART2, buffer, n);
	}
}
