#ifndef __GYRO_TASK_H
#define __GYRO_TASK_H

#include "main.h"
#include "cmsis_os.h"
#include "basic_io.h"
#include "stdio.h"
#include "string.h"
#include "../Drivers/BSP/STM32L476G-Discovery/stm32l476g_discovery_gyroscope.h"

#define GYRO_THRESHOLD_DETECTION (1000)   	/* Tunnng parameters for gyroscope */
#define SAMPLE 100

void gyro_task_init();
void gyro_task(void* argument);

#endif /*__GYRO_H*/
