#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "UART.h"
#include "LED.h"



//static uint8_t buffer[BufferSize];
int j = 0;
extern int STARTPRG;
extern unsigned char UserInput[4];

int main(void)
{
	uint8_t buffer[BufferSize];
	
	__disable_irq();// disables interrupts
	System_Clock_Init(); 
	LED_Init();
	UART2_Init();
	__enable_irq();
	
	int n = 0;
	
	while(1)
	{
		Green_LED_Toggle();
		for(int i = 0; i < 8000000; i++){}
	}
}
