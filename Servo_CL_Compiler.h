#ifndef __SERVO_CL_COMPILER_H
#define __SERVO_CL_COMPILER_H

#include <stdio.h>
#include <stdlib.h>

#include "stm32l476xx.h"
#include "Stack.h"

struct ServoCallStack* createServoCallStack(uint8_t* commands, int size);
int executeNextInstruction(struct ServoCallStack* recipie);

#endif  /*__SERVO_CL_COMPILER_H*/