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
#include "customer_pool.h"
#include <string.h>
#include <stdbool.h>

#define QUEUE_SIZE 100
struct Customer_Ts 
{

	uint32_t customer_ET; //arrival time
	uint32_t customer_LT; //leave queue time
	uint32_t customer_TT; //hold the max value of that
	uint8_t pool_index;
	bool isFree; 

};
//void vCustomer_Gen( void *pvParamters);
//void queue_init(void);
void vTeller(void *pvParameters);
void vMetric(void *pvParameters);
char * displayTellerState(TaskHandle_t hTeller);
#endif /*__BANK_SIMULATION_H*/