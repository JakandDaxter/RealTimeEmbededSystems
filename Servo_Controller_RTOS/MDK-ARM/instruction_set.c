#include "instruction_set.h"

struct Instruction_Set* createInstructionSet(QueueHandle_t q, uint8_t * commands, TaskHandle_t t, int len)
{
	struct Instruction_Set* iset = (struct Instruction_Set*) malloc(sizeof(struct Instruction_Set));
	iset->index = 0;
	iset->commands = commands;
	iset->hq = q;
	iset->ht = t;
  iset->len = len;
	return iset;
}