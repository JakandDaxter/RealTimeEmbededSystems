
#include "stm32l476xx.h"

void Init_Timer2(uint32_t arr);
void Start_Timer2(void);
void Init_GPIO(void);
void TIM2_IRQHandler  (void) ;
uint32_t get_delta_time(int i);
