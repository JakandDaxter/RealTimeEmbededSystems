#include "bank_simulation.h"
#define BUFFER_SIZE 128
#define UART_TIMEOUT 100000

extern UART_HandleTypeDef huart2;

extern TaskHandle_t hteller_1;
extern TaskHandle_t hteller_2;
extern TaskHandle_t hteller_3;

int cs_count_teller_1;
int cs_count_teller_2;
int cs_count_teller_3;

int total_time_serving_customers;

extern SemaphoreHandle_t mutex;

QueueHandle_t hqueue = NULL;

void queue_init()
{
	hqueue = xQueueCreate(QUEUE_SIZE, 4);
}

void vCustomer_Gen( void *pvParamters)
{
	if(hqueue == NULL)
	{
		queue_init();
	}
	uint32_t f; //the random number seed
	uint32_t       Customer_Arrival;
	uint32_t         Customer_TT; //transation time
	BaseType_t xStatus;
    for(;;)
	{
		f = (RNG ->DR);
        Customer_TT = (f)%80 + 5; // 30s to 8mins
        Customer_Arrival = (f)%40 + 10; // 1 to 4mins
        
		xStatus = xQueueSendToBack(hqueue, &Customer_TT, 0);
		if(xStatus == pdPASS)
		{
			vTaskDelay(Customer_Arrival);
		}
	}
}

void vTeller(void *pvParameters)
{
	uint32_t customer_tt;
	uint8_t buffer[BUFFER_SIZE];
	struct Teller* teller = (struct Teller*)pvParameters;
	for(;;)
	{
		if(hqueue != 0)
		{
			if(xQueueReceive(hqueue, &customer_tt, 10) == pdPASS)
			{
				updateTeller(teller, customer_tt);
				if( xSemaphoreTake( mutex, ( TickType_t ) 10 ) == pdTRUE )
        {
					switch(teller->id)
					{
						case 1:
							cs_count_teller_1 = getTellerCS(teller);
							total_time_serving_customers += getTellerTSC(teller);
							break;
						case 2:
							cs_count_teller_2 = getTellerCS(teller);
						case 3:
							cs_count_teller_3 = getTellerCS(teller);
					}
					xSemaphoreGive(mutex);
				}
				vTaskDelay(customer_tt);
			}
		}
	}
}

char * displayTellerState(TaskHandle_t hTeller)
{
	eTaskState t_state = eTaskGetState(hTeller);
	switch(t_state)
	{
		case eRunning:
			return "RUN ";
		case eReady:
			return "IDLE";
		case eBlocked:
			return "BUSY";
		case eSuspended:
			return "SPD ";
		case eDeleted:
			return "DEL ";
		case eInvalid:
			return "INV ";
	}
}

void vMetric(void *pvParameters)
{
	uint8_t buffer[BUFFER_SIZE];
	uint16_t n = 0;
	uint32_t queue_size = 0;
	uint32_t sim_time, sim_time_hours, sim_time_mins = 0;
	
	char* teller_1_state = displayTellerState(hteller_1);
	char* teller_2_state = displayTellerState(hteller_2);
	char* teller_3_state = displayTellerState(hteller_3);
	
	for(;;){
		if(hqueue != 0)
		{
			queue_size = uxQueueMessagesWaiting(hqueue);
		}
		sim_time = xTaskGetTickCount();
		sim_time_hours = (sim_time / 600) + 9;
		sim_time_mins = (sim_time / 10) % 60;
		
		teller_1_state = displayTellerState(hteller_1);
		teller_2_state = displayTellerState(hteller_2);
		teller_3_state = displayTellerState(hteller_3);
		if( xSemaphoreTake( mutex, ( TickType_t ) 10 ) == pdTRUE )
    {  
			n = sprintf((char *)buffer, "Queue size = %d \t Teller 1 %s Customers Served = %d \tTeller 2 %s Customers Served = %d \tTeller 3 %s Customers Served = %d \tSimulation Time = %d:%d \r", 
					queue_size, 
					teller_1_state,
					cs_count_teller_1,
					teller_2_state,
					cs_count_teller_2,
					teller_3_state,
					cs_count_teller_3,
					sim_time_hours, 
					sim_time_mins);
			HAL_UART_Transmit(&huart2, buffer, n, UART_TIMEOUT);
			xSemaphoreGive(mutex);
		}
		if(sim_time_hours == 16)
		{
			vTaskSuspendAll();
			int total_customers_served = (cs_count_teller_1 + cs_count_teller_2 + cs_count_teller_3);
			int avg_customer_tt = ((total_time_serving_customers / total_customers_served)*3) /10;
			//int avg_customer_wait = 
			int avg_teller_idle_time = (4200 - total_time_serving_customers) / 30;
			int max_customer_wait;
			
			n = sprintf((char *)buffer, 
				"\n\rTotal customers served = %d \n\rAverage time customers spend with teller = %d mins \n\rAverage time tellers wait for customers = %d mins \n\rTeller 1 served %d customers\n\rTeller 2 served %d customers\n\rTeller 3 served %d customers\n\r",
					total_customers_served,
					avg_customer_tt, 
					avg_teller_idle_time,
					cs_count_teller_1,
					cs_count_teller_2,
					cs_count_teller_3);
			HAL_UART_Transmit(&huart2, buffer, n, UART_TIMEOUT);
		}
		vTaskDelay(10); //update every minute of simulation time

	}	
}
