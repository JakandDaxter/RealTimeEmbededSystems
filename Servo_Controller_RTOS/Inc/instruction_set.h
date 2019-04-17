#ifndef __INSTRUCTION_SET_H
#define __INSTRUCTION_SET_H
#include <stdio.h>
#include <stdlib.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"

struct Instruction_Set
{
    int index;
    uint8_t * commands;
    QueueHandle_t hq;
		TaskHandle_t ht;
    int len;
};

struct Instruction_Set* createInstructionSet(QueueHandle_t q, uint8_t * commands, TaskHandle_t t, int len);
#endif /*__INSTRUCTION_SET_H*/