#ifndef __TIMER_3_PIT_H
#define __TIMER_3_PIT_H

#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"

#define PERIOD 200; //500us * PERIOD = Period of Timer 3  

void Timer3_Init(void);

#endif /*__TIMER_3_PIT_H*/