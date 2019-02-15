#ifndef __STACK_H 
#define __STACK_H

#include <stdio.h> 
#include <stdlib.h> 
#include <limits.h>

#include "stm32l476xx.h"

struct Stack 
{ 
    uint8_t top;
    unsigned capacity; 
    uint8_t* array; 
}; 
struct Stack* createCallStack(uint8_t* commands, int size);
struct Stack* createStack(unsigned capacity);
int isFull(struct Stack* stack);
int isEmpty(struct Stack* stack); 
void push(struct Stack* stack, uint8_t item);
uint8_t pop(struct Stack* stack);

#endif /*__STACK_H*/