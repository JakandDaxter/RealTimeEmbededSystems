#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "UART.h"
#include "LED.h"
#include "States.h"
#include "Timer_3_PIT.h"
#include "Servo_CL_Compiler.h"
#include "Servo.h"

extern struct Servo* servo_1;
extern struct Servo* servo_2;

int main(void)
{
	__disable_irq();// disables interrupts
	System_Clock_Init(); 
	LED_Init();
	UART2_Init();
	Timer3_Init();
	__enable_irq();
	unsigned char UserInput[10];
	Print("Welcome to the Servo Control system\r\n");
	Print("The system simultaneously controls two servos.\n\rEach servo has a dedicated command script that has been preloaded.\n\r");
	Print("You are able to START, PAUSE, and CONTINUE the execution.\n\rOr move either servo LEFT or RIGHT 1 position.\n\r");
	while(1)
	{
		int j = 0;
		memset(&UserInput,0,sizeof(UserInput));
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
			start_servo(servo_1);
		}
		else if((UserInput[0] == 'P') || (UserInput[0] == 'p'))
		{
			pause_servo(servo_1);
		}
		else if((UserInput[0] == 'C') || (UserInput[0] == 'c'))
		{
			continue_servo(servo_1);
		}
		else if((UserInput[0] == 'N') || (UserInput[0] == 'n'))
		{
			
		}		
		else{Print("Override Not Available\n\r");}
		/*Servo 2 control*/
		
		if((UserInput[1] == 'B') || (UserInput[1] == 'b'))
		{
			start_servo(servo_2);	
		}
		else if((UserInput[1] == 'P') || (UserInput[1] == 'p'))
		{
			pause_servo(servo_2);
		}
		else if((UserInput[1] == 'C') || (UserInput[1] == 'c'))
		{
			continue_servo(servo_2);
		}
		else if((UserInput[1] == 'N') || (UserInput[1] == 'n'))
		{
			
		}
		else{Print("Override Not Available\n\r");}
	}
}
