#include "multitasking_servo_driver.h"

int main(void)
{
	/*
	initializes the baremetal hardware as well as loads default recipie into servoes 1 and 2
	intialization includes:
		USART2
		LEDS
		TIMER2
		TIMER3
		SYSCLOCK
	*/
	init_multitasking_servo_driver();
	
	
	Print("Welcome to the Servo Control system\r\n");
	Print("The system simultaneously controls two servos.\n\rEach servo has a dedicated command script that has been preloaded.\n\r");
	Print("You are able to START, PAUSE, and CONTINUE the execution.\n\rOr move either servo LEFT or RIGHT 1 position.\n\r");
	while(1)
	{
		evaluate_overrides(); //non blocking loop to allow user overriedes
		 //leds light up to indicate status of execution of recipies on servo1
		//status
		//Green LED on = running 
		//Red LED on = command error
		//Both on = nested loop error
		//Both off = paused
	}
}
