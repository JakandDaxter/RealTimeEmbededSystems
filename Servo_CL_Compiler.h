#ifndef __SERVO_CL_COMPILER_H
#define __SERVO_CL_COMPILER_H

#include <stdio.h>
#include <stdlib.h>

#include "stm32l476xx.h"
#include "UART.h"
#include "Servo.h"


struct ServoCallStack
{
	uint8_t top;
	unsigned capacity;
	uint8_t* array;
	int loopCount;
	uint8_t loopPointer;
	uint8_t pausePointer;
	int depth;
};

int isEmpty(struct ServoCallStack* stack);
int isFull(struct ServoCallStack* stack);
void push(struct ServoCallStack* stack, uint8_t item);
uint8_t pop(struct ServoCallStack* stack);
struct ServoCallStack* createServoCallStack(uint8_t* commands, int size);

#endif  /*__SERVO_CL_COMPILER_H*/