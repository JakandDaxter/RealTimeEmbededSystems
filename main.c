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

int main(void)
{
	WELCOMEMESSAGE();
	while(STARTPRG == 1){ //so unless they say start the program it wont start
	__disable_irq();// disables interrupts
	System_Clock_Init(); 
	Init_GPIO();
	LED_Init();
	Init_Timer2(8000000U);
	UART2_Init();
	__enable_irq();
	while(1){if(POST()==0)break;}
}

		USART_Write(USART2, (uint8_t *)"Exitting Program......\r\n\r\n\n\n\n\n", 20);

		return (0); //so it will end the program	
	
}