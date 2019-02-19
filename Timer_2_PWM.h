#ifndef __TIMER_2_INPUT_CAPTURE_H
#define __TIMER_2_INPUT_CAPTURE_H

#include "stm32l476xx.h"

void Init_Timer2();
void Start_Timer2(void);
void Stop_Timer2(void);
void Init_GPIO(void);
//void TIM2_IRQHandler  (void) ;

#endif /*__TIMER_2_INPUT_CAPTURE_H*/ 