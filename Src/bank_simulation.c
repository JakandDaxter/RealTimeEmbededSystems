#include "bank_simulation.h"
#define BUFFER_SIZE 128
#define UART_TIMEOUT 100000

extern UART_HandleTypeDef huart2;
extern SemaphoreHandle_t mutex;
extern struct Teller* tellers[3];

extern TaskHandle_t hteller_1;
extern TaskHandle_t hteller_2;
extern TaskHandle_t hteller_3;

QueueHandle_t hqueue = NULL;

int cs_count_teller_1 = 0, cs_count_teller_2 = 0, cs_count_teller_3 = 0;
int idle_time_t1, idle_time_t2, idle_time_t3, max_idle_t1 = 0, max_idle_t2 = 0, max_idle_t3 = 0;
float total_wait_time_c = 0, max_wait_time_c = 0;
void queue_init()
{
	hqueue = xQueueCreate(QUEUE_SIZE, sizeof(int));
}

void vTeller(void *pvParameters)
{
	uint32_t f,teller_idle_stop, n;
	uint8_t buffer[BUFFER_SIZE];
	float customer_wait_time = 0;
	struct Teller* teller = (struct Teller*)pvParameters;
	char* teller_1_state = displayTellerState(hteller_1);
	char* teller_2_state = displayTellerState(hteller_2);
	char* teller_3_state = displayTellerState(hteller_3);
	
	struct Customer_Ts* customer;
	
	TickType_t tick ;
	
    for(;;)
    {
		if(hqueue != 0)
		{
			if(xQueueReceive(hqueue, &customer, (TickType_t)10) == pdPASS)
			{
				if( xSemaphoreTake( mutex, ( TickType_t ) 20000 ) == pdTRUE )
				{
					
					updateTeller(teller, customer->customer_TT);
					tick = xTaskGetTickCount();
					customer_wait_time = (xTaskGetTickCount() - customer->customer_ET)/100;
					total_wait_time_c = customer_wait_time + total_wait_time_c;
					if(customer_wait_time > max_wait_time_c)
					{
						max_wait_time_c = customer_wait_time;
					}	
					free_customer(customer->pool_index);
					switch(teller->id)
					{
						case 1:
							cs_count_teller_1 = getTellerCS(teller);
							idle_time_t1 = (tick - teller->prev_busy_tick_count - customer->customer_TT);
							teller->prev_busy_tick_count = tick;
                            if(max_idle_t1 < idle_time_t1)
                            {
                                max_idle_t1 = idle_time_t1;
                            }
							break;
						case 2:
							cs_count_teller_2 = getTellerCS(teller);
							idle_time_t2 = (tick - teller->prev_busy_tick_count - customer->customer_TT);
							teller->prev_busy_tick_count = tick;
                            if(max_idle_t2 < idle_time_t2)
                            {
                                max_idle_t2 = idle_time_t2;
                            }
							break;
						case 3:
							cs_count_teller_3 = getTellerCS(teller);
							idle_time_t3 = (tick - teller->prev_busy_tick_count - customer->customer_TT);
							teller->prev_busy_tick_count = tick;
                            if(max_idle_t3 < idle_time_t3)
                            {
                                max_idle_t3 = idle_time_t3;
                            }
							break;
					}
					n = sprintf((char *)buffer, "\t\t\t\t\t T1 = %d %s \t T2 = %d %s \t T3 = %d %s \r", cs_count_teller_1, teller_1_state,
						cs_count_teller_2, teller_2_state,
						cs_count_teller_3, teller_3_state);
					HAL_UART_Transmit(&huart2, buffer, n, UART_TIMEOUT);
					xSemaphoreGive(mutex);
					vTaskDelay(customer->customer_TT);
				}
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
	uint32_t queue_size, f, n, ticks, sim_hours, sim_mins;
	uint8_t max_q_size = 0;
	uint8_t buffer[BUFFER_SIZE];
	int Customer_TT, Customer_Arrival;
	float avg_customer_wait_time = 0;
	float avg_teller_wait_time = 0;
	float max_transaction_time = 0;
	if(hqueue == NULL)
	{
		queue_init();
	}
	
    for(;;)
    {
		struct Customer_Ts* customer = find_free_customer();
		f = (RNG ->DR);
        Customer_TT = (f)%750 + 50; // 30s to 8mins
        Customer_Arrival = (f)%300 + 100; // 1 to 4mins
		customer->customer_ET = xTaskGetTickCount();
		customer->customer_TT = Customer_TT;
		customer->isFree = false;
		BaseType_t xStatus = xQueueSendToBack(hqueue, &customer, (TickType_t) 10);
        float max_time_tellers_wait = 0;
				if(getTellerMaxTransactionTime(tellers[0]) > getTellerMaxTransactionTime(tellers[1]))
        {
            max_transaction_time = getTellerMaxTransactionTime(tellers[0]);
        }
        else
        {
            max_transaction_time = getTellerMaxTransactionTime(tellers[1]);
        }
        if(max_transaction_time > getTellerMaxTransactionTime(tellers[2]))
        {
            max_transaction_time = getTellerMaxTransactionTime(tellers[2]);
        }
			////////
        if(max_idle_t1 > max_idle_t2)
        {
            max_time_tellers_wait = max_idle_t1;
        }
        else
        {
            max_time_tellers_wait = max_idle_t2;
        }
        if(max_time_tellers_wait > max_idle_t3)
        {
            max_time_tellers_wait = max_idle_t3;
        }
        max_time_tellers_wait = max_time_tellers_wait / 100;
		if(xStatus == pdPASS)
        {
			ticks = xTaskGetTickCount();
			sim_hours = (ticks/6000) + 9;
			sim_mins = (ticks/100) % 60;
			queue_size = uxQueueMessagesWaiting(hqueue);
			if(queue_size > max_q_size)
			{
				max_q_size = queue_size;
			}
			n = sprintf((char *)buffer, "Queue size = %d \tSimulation time %d:%d \r", queue_size, sim_hours, sim_mins);
			if( xSemaphoreTake( mutex, ( TickType_t ) 1000 ) == pdTRUE )
			{
				HAL_UART_Transmit(&huart2, buffer, n, UART_TIMEOUT);
				xSemaphoreGive(mutex);
			}
			if(sim_hours >= 16)
			{
				vTaskSuspendAll();
				int total_cs = cs_count_teller_1 + cs_count_teller_2 + cs_count_teller_3;
        float average_transaction_time = ((getTellerTSC(tellers[0]) +getTellerTSC(tellers[1])+getTellerTSC(tellers[2]))/total_cs)/100;				
				avg_customer_wait_time = total_wait_time_c / total_cs;
				avg_teller_wait_time = ((idle_time_t1 + idle_time_t2 + idle_time_t3)/300);
				n = sprintf((char *)buffer, "\n\rMax queue size = %d \n\r"
					"Total customers served = %d \n\r" 
					"Average time each customer spends in the queue = %.4f \n\r"
					"Max time customer spends in queue = %.4f \n\r"
					"Average time tellers wait for customers = %.4f \n\r"
          "Max time tellers wait for customers = %.4f mins \n\r"
				"Max time Transaction Time for Tellers: %.4f mins \n\r"
				"Average Teller Transaction Time: %.4f mins \n\r",
					max_q_size,
					total_cs,
					avg_customer_wait_time,
					max_wait_time_c,
					avg_teller_wait_time ,
          max_time_tellers_wait,
          (max_transaction_time/100),
						average_transaction_time);
				HAL_UART_Transmit(&huart2, buffer, n, UART_TIMEOUT);
			}
			vTaskDelay(Customer_Arrival);
		}
    }
}
