#include "Servo_CL_Compiler.h"
#include "States.h"
#include "Stack.h"
#include "UART.h"

#define PARAM 0x1F
#define OP 0xE0

struct ServoCallStack
{
	struct Stack* stack;
	uint8_t loopPointer;
	int loopCount;
};

struct ServoCallStack* createServoCallStack(uint8_t* commands, int size)
{
	struct ServoCallStack* servoStack = (struct ServoCallStack*) malloc(sizeof(struct ServoCallStack));
	servoStack->stack = createCallStack(commands, size);
	servoStack->loopCount = -1;
	servoStack->loopPointer = NULL;
    return servoStack;	
}

int executeNextInstruction(struct ServoCallStack* recipie)
{
	uint8_t instruction = pop(recipie->stack);
	uint8_t op = instruction & OP;
	uint8_t parameter = instruction & PARAM;
	if(instruction == 0xFF)
	{
		return -1;
	}
	else
	{
		switch(op)
		{
			case MOV:
				Printf("Move to %d\n\r", parameter);
				break;
			case WAIT:
				if((parameter > 0) && (recipie->loopCount < 0)){
					recipie->stack->top--;
					Printf("Waiting for %d cycles\n\r", parameter);
					recipie->loopCount = parameter - 1;
					return 0;
				}
				else if(recipie->loopCount > 0){
					recipie->stack->top--;
					Printf("Waiting for %d cycles\n\r", recipie->loopCount);
					recipie->loopCount--;
					return 0;
				}
				else if(parameter == 0)
				{
					Printf("Waiting for %d cycles\n\r", parameter);
					return 0;
				}
				Printf("Waiting for %d cycles\n\r", recipie->loopCount);
				recipie->loopCount--;
				return 0;
			case LOOP:
				recipie->loopPointer = recipie->stack->top;
				recipie->loopCount = parameter;
				Printf("Loop pointer saved. Top of loop is 0x%X\r\n", recipie->loopPointer);
				return 0;
			case END_LOOP:
				if(recipie->loopCount >= 0)
				{
					recipie->stack->top = recipie->loopPointer;
					recipie->loopCount--;
				}
				return 0;
			default: Print("Command Error!\n\r");
		}
		return 0;
	}
}