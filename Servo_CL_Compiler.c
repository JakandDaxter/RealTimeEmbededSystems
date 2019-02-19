#include "Servo_CL_Compiler.h"
#include "States.h"
#include "UART.h"

// Stack is full when top is equal to the last index 
int isFull(struct ServoCallStack* stack) 
{   return stack->top == 0; } 
  
// Stack is empty when top is equal to -1 
int isEmpty(struct ServoCallStack* stack) 
{   return stack->top == (stack->capacity);  } 
  
// Function to add an item to stack.  It decreases top by 1 
void push(struct ServoCallStack* stack, uint8_t item) 
{ 
    if (isFull(stack)) 
        return; 
    stack->array[--stack->top] = item; 
} 
  
// Function to remove an item from stack.  It increases top by 1 
uint8_t pop(struct ServoCallStack* stack) 
{ 
    if (isEmpty(stack)) 
        return 0xFF; 
    return stack->array[stack->top++]; 
}

struct ServoCallStack* createServoCallStack(uint8_t* commands, int size)
{
	struct ServoCallStack* servoStack = (struct ServoCallStack*) malloc(sizeof(struct ServoCallStack));
	servoStack->loopCount = -1;
	servoStack->loopPointer = NULL;
	servoStack->pausePointer = NULL;
	servoStack->top = size;
    servoStack->capacity = size; 
    servoStack->array = commands;
	servoStack->depth = 0;
    return servoStack;	
}

