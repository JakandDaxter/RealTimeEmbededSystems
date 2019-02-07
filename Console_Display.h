#ifndef __Console_Display_H
#define __Console_Display_H

#include "stm32l476xx.h"
#include <string.h>
#include "UART.h"
#define sample 1001

int POSTFAIL(void);
int POST(void);
int reruns(void);
void SORTER(int array[]);
int WELCOMEMESSAGE(void);

#endif /*__Console_Display_H*/ 
