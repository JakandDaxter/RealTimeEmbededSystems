#ifndef __TIMER_2_INPUT_CAPTURE_H
#define __TIMER_2_INPUT_CAPTURE_H

#include "stm32l476xx.h"
#define ARRAY_SIZE 1000

void Init_Timer2(uint32_t arr);
void Start_Timer2(void);
void Init_GPIO(void);
void TIM2_IRQHandler  (void) ;
uint32_t get_delta_time(int i);

#endif /*__TIMER_2_INPUT_CAPTURE_H*/ 
