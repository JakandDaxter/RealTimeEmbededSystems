#ifndef __TIMER_2_PWM_H
#define __TIMER_2_PWM_H
#include <string.h>
#include <stdio.h>

#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "Servo.h"

void Init_Timer2();
void Start_Timer2(void);
void Stop_Timer2(void);
void Init_GPIO(void);
void move_to_position(struct Servo* servo, int position);
//void TIM2_IRQHandler  (void) ;

#endif /*__TIMER_2_PWM_H*/ 