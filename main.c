#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "UART.h"
#include "LED.h"
#include "States.h"
#include "Timer_3_PIT.h"
#include "Stack.h"
#include "Servo_CL_Compiler.h"
#include "Timer_5_PIT.h"

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
	start_timer5();
	unsigned char UserInput[10];
	Print("Welcome to the Servo Control system\r\n");
	Print("The system simultaneously controls two servos.\n\rEach servo has a dedicated command script that has been preloaded.\n\r");
	Print("You are able to START, PAUSE, and CONTINUE the execution.\n\rOr move either servo LEFT or RIGHT 1 position.\n\r");
	while(1)
	{
		int j = 0;
		Red_LED_Off();
		Green_LED_Off();
		Print(">");
		unsigned char USART_char = '0';
		while(USART_char != '\r')
		{
			USART_char = USART_Read(USART2);
			USART_Write(USART2,&USART_char,1);
			UserInput[j] = USART_char;
			j++;
		}
		Print("\n\r");
		if((UserInput[0] == 'B') || (UserInput[0] == 'b'))
		{
			start_timer3();	
		}
		else if((UserInput[0] == 'P') || (UserInput[0] == 'p'))
		{
			stop_timer3();
		}
		else if((UserInput[0] == 'C') || (UserInput[0] == 'c'))
		{
			restart_timer3();
		} 
		else{Print("Override Not Available\n\r");}
		/*Servo 2 control*/
		
		if((UserInput[1] == 'B') || (UserInput[1] == 'b'))
		{
			start_timer5();	
		}
		else if((UserInput[1] == 'P') || (UserInput[1] == 'p'))
		{
			stop_timer5();
		}
		else if((UserInput[1] == 'C') || (UserInput[1] == 'c'))
		{
			restart_timer5();
		} 
		else{Print("Override Not Available\n\r");}
	}
}
