#include "Stack.h"
  
// function to create a stack of given capacity. It initializes size of 
// stack as 0 
struct Stack* createStack(unsigned capacity) 
{ 
    struct Stack* stack = (struct Stack*) malloc(sizeof(struct Stack)); 
    stack->capacity = capacity; 
    stack->top = capacity; 
    stack->array = (uint8_t*) malloc(stack->capacity * sizeof(uint8_t)); 
    return stack; 
} 

struct Stack* createCallStack(uint8_t* commands, int size)
{
	struct Stack* stack = (struct Stack*) malloc(sizeof(struct Stack)); 
    stack->capacity = size; 
    stack->top = 0;
    stack->array = commands; 
    return stack; 
}
  
// Stack is full when top is equal to the last index 
int isFull(struct Stack* stack) 
{   return stack->top == 0; } 
  
// Stack is empty when top is equal to -1 
int isEmpty(struct Stack* stack) 
{   return stack->top == (stack->capacity);  } 
  
// Function to add an item to stack.  It decreases top by 1 
void push(struct Stack* stack, uint8_t item) 
{ 
    if (isFull(stack)) 
        return; 
    stack->array[--stack->top] = item; 
} 
  
// Function to remove an item from stack.  It increases top by 1 
uint8_t pop(struct Stack* stack) 
{ 
    if (isEmpty(stack)) 
        return 0xFF; 
    return stack->array[stack->top++]; 
} 


