#include "bank_simulation.h"
#define BUFFER_SIZE 128
#define UART_TIMEOUT 100000

struct queue
{
	int size;
};

struct queue q;
///////////////////////////////////
extern UART_HandleTypeDef huart2;

extern TaskHandle_t hteller_1;
extern TaskHandle_t hteller_2;
extern TaskHandle_t hteller_3;

int cs_count_teller_1;
int cs_count_teller_2;
int cs_count_teller_3;

int queue_size;

int total_wait_time;
uint32_t max_wait_time_c, max_wait_time_t;

int total_time_serving_customers;
int exittime,arrivetime,max_transaction_time = 0;

extern SemaphoreHandle_t mutex;

QueueHandle_t hqueue = NULL;

void queue_init()
{
	hqueue = xQueueCreate(QUEUE_SIZE, 4);
}

void vCustomer_Gen( void *pvParamters)
{
	uint32_t n;
	int queue_size = 0;
	uint8_t buffer[BUFFER_SIZE];
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
		struct Customer_Ts* customer = find_free_customer();
		f = (RNG ->DR);
        Customer_TT = (f)%80 + 5; // 30s to 8mins
        Customer_Arrival = (f)%40 + 10; // 1 to 4mins
        //updateEnter(customer, xTaskGetTickCount()); //update entered time
				//updateTransationtime(customer,Customer_TT);
				customer->customer_ET = TIM3->CNT;
				customer->customer_TT = Customer_TT;
		xStatus = xQueueSendToBack(hqueue, &customer, 0);
		if(xStatus == pdPASS)
		{
			q.size++;
			n = sprintf((char*)buffer,"Queue size = %d \r", q.size);
			HAL_UART_Transmit(&huart2, buffer, n, UART_TIMEOUT);
			vTaskDelay(Customer_Arrival);
		}
	}
}

void vTeller(void *pvParameters)
{
	uint32_t n, customer_wait_time,teller_idle_stop;
	uint8_t buffer[BUFFER_SIZE];
	struct Teller* teller = (struct Teller*)pvParameters;
	struct Customer_Ts* customer;
	for(;;)
	{
		if(hqueue != 0)
		{
			if(xQueueReceive(hqueue, &customer, 10) == pdPASS)
			{
				//q.size--;
				if( xSemaphoreTake( mutex, ( TickType_t ) 10 ) == pdTRUE )
				{
					q.size--;
					customer_wait_time = (TIM3->CNT - customer->customer_ET)/100;
					if(customer_wait_time > max_wait_time_c)
					{
						max_wait_time_c = customer_wait_time;
					}	
					updateTeller(teller, customer->customer_TT);
					if(customer->customer_TT > max_transaction_time)
					{
						max_transaction_time = customer->customer_TT;
					}
					switch(teller->id)
					{
						case 1:
							cs_count_teller_1 = getTellerCS(teller);
							total_time_serving_customers += getTellerTSC(teller);
							max_wait_time_t = getTellerMaxWaitTime(teller);
							break;
						case 2:
							cs_count_teller_2 = getTellerCS(teller);
						case 3:
							cs_count_teller_3 = getTellerCS(teller);
					}
					xSemaphoreGive(mutex);
				}
				free_customer(customer->pool_index);
				vTaskDelay(customer->customer_TT);
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
	uint16_t n = 0, teller_idle_start = 0, teller_idle_stop = 0, teller_idle_time = 0;
	uint32_t queue_size = 0;
	uint32_t queue_max = 0;
	uint32_t sim_time, sim_time_hours, sim_time_mins = 0;
	struct Customer_Ts customer;
	char* teller_1_state = displayTellerState(hteller_1);
	char* teller_2_state = displayTellerState(hteller_2);
	char* teller_3_state = displayTellerState(hteller_3);
	
	for(;;){
		if(hqueue != 0)
		{
			queue_size = uxQueueMessagesWaiting(hqueue);
			if(queue_size > queue_max)
			{
				queue_max = queue_size;
			}
		}
		sim_time = xTaskGetTickCount();
		sim_time_hours = (sim_time / 600) + 9;
		sim_time_mins = (sim_time / 10) % 60;
		//Customer_Maxupdate(customer);//update max time
		teller_1_state = displayTellerState(hteller_1);
		teller_2_state = displayTellerState(hteller_2);
		teller_3_state = displayTellerState(hteller_3);
		if( xSemaphoreTake( mutex, ( TickType_t ) 10 ) == pdTRUE )
    {  
			n = sprintf((char *)buffer, "Queue size = %d \t Teller 1 %s Customers Served = %d \tTeller 2 %s Customers Served = %d \tTeller 3 %s Customers Served = %d \tSimulation Time = %d:%d \r", 
					q.size, 
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
			int avg_customer_wait = total_wait_time / total_customers_served;
			int avg_teller_idle_time = (4200 - total_time_serving_customers) / 30;
			//int max_TT_time = getMaxTT(customer);
			n = sprintf((char *)buffer, 
				"\n\rTotal customers served = %d \n\rTeller 1 served %d customes \tTeller 2 served %d customers \tTeller 3 served %d customers \n\r"
				"Average time customer spends in queue = %d mins \n\r"
				"Average time customer spends with teller = %d mins \n\r"
				"Average time tellers wait for customers = %d mins \n\r"
				"Max time customer spends in queue = %d mins \n\r"
				"Max time tellers wait for customers = %d mins \n\r"
				"Max transaction time = %d mins \n\r"
				"Max size of queue = %d customers \n\r",
				total_customers_served, cs_count_teller_1, cs_count_teller_2, cs_count_teller_3,
				avg_customer_wait,
				avg_customer_tt,
				avg_teller_idle_time,
				max_wait_time_c/10,
				max_wait_time_t/10,
				max_transaction_time/10,
				queue_max);
			HAL_UART_Transmit(&huart2, buffer, n, UART_TIMEOUT);
		}
		vTaskDelay(10); //update every minute of simulation time

	}	
}
