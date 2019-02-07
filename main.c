#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "UART.h"
#include "LED.h"
#include "Timer_2_Input_Capture.h"
#include "Console_Display.h"

//static uint8_t buffer[BufferSize];
int j = 0;
extern int STARTPRG;

int main(void)
{
	
	
	__disable_irq();// disables interrupts
	System_Clock_Init(); 
	Init_GPIO();
	LED_Init();
	Init_Timer2(8000000U);
	UART2_Init();
	__enable_irq();
	while(1)
	{
		USART_Write(USART2, (uint8_t*)"Press enter to continue\n\r", 25);
		unsigned char USART_char = USART_Read(USART2);
		while(USART_char != '\r'){}
		USART_Write(USART2, &USART_char, 1);
		USART_Write(USART2, (uint8_t *)"\n\r", 2);
		WELCOMEMESSAGE();
		while(STARTPRG == 1)
		{ //so unless they say start the program it wont start}
			USART_Write(USART2, (uint8_t *)"Exitting Program......\r\n\r\n\n\n\n\n", 20);
			return (0); //so it will end the program	
		}
	}
}
