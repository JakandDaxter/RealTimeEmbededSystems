#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "UART.h"
#include "LED.h"
#include "Timer_3_PIT.h"
#include "Stack.h"

uint8_t buffer[BufferSize];

int main(void)
{
	uint8_t buffer[BufferSize];
	__disable_irq();// disables interrupts
	System_Clock_Init(); 
	LED_Init();
	UART2_Init();
	Timer3_Init();
	__enable_irq();
	
	int n = 0;
	
	struct StackNode* root = NULL;
	push(&root, 10);
	push(&root, 20);
	push(&root, 30);
	
	n = sprintf((char *)buffer, "top = %d\t", pop(&root));
	USART_Write(USART2,buffer,n);
	n = sprintf((char *)buffer, "new top = %d\r\n",peek(root));
	USART_Write(USART2,buffer,n);
	while(1)
	{
	}
}
