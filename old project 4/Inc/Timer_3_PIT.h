#ifndef __TIMER_3_PIT_H
#define __TIMER_3_PIT_H

#include "stm32l476xx.h"

#define PERIOD 0x0EA60; 

void Timer3_Init(void);
void start_timer3(int channel);
void stop_timer3(int channel);
void restart_timer3(int channel);
void pause_timer3(int channel);
#endif /*__TIMER_3_PIT_H*/