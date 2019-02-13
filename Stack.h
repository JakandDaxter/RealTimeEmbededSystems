#ifndef __STACK_H 
#define __STACK_H
#include <stdio.h>
#include <stdlib.h>

#include "stm32l476xx.h"

struct StackNode* NewNode(uint8_t command);
int isEmpty(struct StackNode* root);
void push(struct StackNode** root, uint8_t command);
uint8_t pop(struct StackNode** root);
uint8_t peek(struct StackNode* root);

#endif /*__STACK_H*/