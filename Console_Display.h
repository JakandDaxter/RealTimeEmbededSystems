#ifndef __Console_Display_H
#define __Console_Display_H

#include "stm32l476xx.h"
#include <string.h>
#include "UART.h"
#define sample 1001

int POSTFAIL(void);
int POST(void);
void reruns(void);
void SORTER(int array[]);

#endif /*__Console_Display_H*/ 
