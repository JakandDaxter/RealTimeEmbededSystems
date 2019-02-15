#ifndef __TIMER_5_PIT_H
#define __TIMER_5_PIT_H

#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"

void load_recipie_servo2(void);
void start_timer5(void);
void stop_timer5(void);
void restart_timer5(void);
void Timer5_Init(void);
#define PERIOD 800; 
#endif /*__TIMER_5_PIT_H*/