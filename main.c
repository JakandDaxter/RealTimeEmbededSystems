#include <string.h>
#include <stdio.h>
#include <ctype.h>

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
			UserInput[j] = (uint8_t)toupper(USART_char);
			j++;
		}
		Print("\n\r");
		if(UserInput[j-2] != 'X')
		{
			if(UserInput[0] == 'B')
			{
				start_servo(servo_1);
				
			}
			else if(UserInput[0] == 'P')
			{
				pause_servo(servo_1);
			}
			else if(UserInput[0] == 'C')
			{
				continue_servo(servo_1);
			}
			else if(UserInput[0] == 'N')
			{
				
			}
			else{Print("Override Not Available\n\r");}
			/*Servo 2 control*/
			
			if(UserInput[1] == 'B')
			{
				start_servo(servo_2);	
			}
			else if(UserInput[1] == 'P')
			{
				pause_servo(servo_2);
			}
			else if(UserInput[1] == 'C')
			{
				continue_servo(servo_2);
			}
			else if(UserInput[1] == 'N')
			{
				
			}
			else{Print("Override Not Available\n\r");}
		}
		
		switch(servo_1->status)
		{
			case status_running:
				Green_LED_On();
				Red_LED_Off();
				break;
			case status_command_error:
				Red_LED_On();
				Green_LED_Off();
				break;
			case status_nested_error:
				Red_LED_On();
				Green_LED_On();
				break;
			case status_paused:
				Green_LED_Off();
				Red_LED_Off();
				break;
			default:
				Green_LED_Off();
				Red_LED_Off();
		}
	}
}
