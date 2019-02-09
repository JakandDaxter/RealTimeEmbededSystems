#ifndef __Console_Display_H
#define __Console_Display_H

#include "stm32l476xx.h"
#include <string.h>
#include "UART.h"
#define  sample 1001

int POSTFAIL(void);
int POST(void);
int reruns(void);
void SORTER(uint32_t A[], int size);
int WELCOMEMESSAGE(void);
int Parse_Str_To_Int(unsigned char * arrayStart, int sizeOfArray);
int Verify_Input(int input);
int editorORrunner(void);
void Pop_Buckets(int lower_bound);
void Pop_Histogram (int lower_bound);
void Output_Histogram(void);
int rerunFunc( void );
#endif /*__Console_Display_H*/ 
