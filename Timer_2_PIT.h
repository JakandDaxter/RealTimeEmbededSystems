#ifndef __TIMER_3_PIT_H
#define __TIMER_3_PIT_H

#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"

#define PERIOD 800; 

void Timer3_Init(void);
void load_recipie(void);
void start_timer3(void);
void stop_timer3(void);
void restart_timer3(void);
#endif /*__TIMER_3_PIT_H*/