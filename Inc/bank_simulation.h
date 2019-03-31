#ifndef __BANK_SIMULATION_H
#define __BANK_SIMULATION_H
#include <stdio.h>
#include "FreeRTOS.h"
#include "queue.h"
#include "main.h"
#include "cmsis_os.h"
#include "task.h"
#include "teller.h"
#include "customer.h"

#define QUEUE_SIZE 50

void vCustomer_Gen( void *pvParamters);
void queue_init(void);
void vTeller(void *pvParameters);
void vMetric(void *pvParameters);
#endif /*__BANK_SIMULATION_H*/