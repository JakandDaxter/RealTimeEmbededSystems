#ifndef __TIMER_3_PIT_H
#define __TIMER_3_PIT_H

#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"

#define PERIOD 800; 

void Timer3_Init(void);
void load_recipies(void);
void start_timer3(int channel);
void stop_timer3(int channel);
void restart_timer3(int channel);
void pause_timer3(int channel);
#endif /*__TIMER_3_PIT_H*/