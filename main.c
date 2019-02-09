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
extern unsigned char UserInput[4];

int main(void)
{
	uint8_t buffer[BufferSize];
	
	__disable_irq();// disables interrupts
	System_Clock_Init(); 
	Init_GPIO();
	LED_Init();
	Init_Timer2(8000000U);
	UART2_Init();
	__enable_irq();
	
	int n = 0;
	
	while(1)
	{
		if(POST() ==0)break;
	}
}
