#include "bank_simulation.h"
#define BUFFER_SIZE 128
#define UART_TIMEOUT 100000
///////////////////////////////////customer timestamp


////////
void updateEnter(struct Customer_Ts time, uint32_t customer_arival)
{
	time.customer_ET = customer_arival;
}
////////
void updateTransationtime(struct Customer_Ts time, uint32_t customer_tt)
{
	int temp;
	 temp = customer_tt;
	
	if(time.customer_TT <= temp){
		
		time.customer_TT = temp;
	
	}
}
/////////
int getMaxTT(struct Customer_Ts time)
{	
	return time.customer_TT ;
}
/////////
void updateLeave(struct Customer_Ts time, uint32_t customer_left)
{
	time.customer_LT = customer_left;
}
///////
//void Customer_Maxupdate(struct Customer_Ts time)
//{
//	int temp;
//	
//	temp =  (time.customer_LT) - (time.customer_ET) ;
//	
//	if(( time.customer_MT) <= temp)
//	{
//	time.customer_MT = temp;
//	}

//}
/////////// 
//int getMaxTime(struct Customer_Ts time)
//{	
//	return time.customer_MT ;
//}

///////////  recursivly check

struct queuesize 
{

	uint32_t size; //


} queuesize;


void update_MaxQueu_size(struct queuesize lengh, int currentlength)
{
	int temp = 0;
	
	if(currentlength >= temp)
	{ 
	 lengh.size = currentlength;
	}				
	 
	lengh.size = temp;
}

int MaxQueue(struct queuesize lengh)
{
	return lengh.size;	
}

///////////////////////////////////
extern UART_HandleTypeDef huart2;

extern TaskHandle_t hteller_1;
extern TaskHandle_t hteller_2;
extern TaskHandle_t hteller_3;

int cs_count_teller_1;
int cs_count_teller_2;
int cs_count_teller_3;

int total_wait_time;
uint32_t max_wait_time;

int total_time_serving_customers;
int exittime,arrivetime,maxtime;

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
			vTaskDelay(Customer_Arrival);
		}
	}
}

void vTeller(void *pvParameters)
{
	uint32_t n, customer_wait_time;
	uint8_t buffer[BUFFER_SIZE];
	struct Teller* teller = (struct Teller*)pvParameters;
	struct Customer_Ts* customer;
	for(;;)
	{
		if(hqueue != 0)
		{
			if(xQueueReceive(hqueue, &customer, 10) == pdPASS)
			{
				if( xSemaphoreTake( mutex, ( TickType_t ) 10 ) == pdTRUE )
        {
					customer_wait_time = (TIM3->CNT - customer->customer_ET)/100;
					if(customer_wait_time > max_wait_time)
					{
						max_wait_time = customer_wait_time;
					}	
					updateTeller(teller, customer->customer_TT);
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
	uint16_t n = 0;
	uint32_t queue_size = 0;
	uint32_t queue_max = 0;
	uint32_t sim_time, sim_time_hours, sim_time_mins = 0;
	struct Customer_Ts customer;
	struct queuesize currentsize;
	
	char* teller_1_state = displayTellerState(hteller_1);
	char* teller_2_state = displayTellerState(hteller_2);
	char* teller_3_state = "NA  ";//displayTellerState(hteller_3);
	
	for(;;){
		if(hqueue != 0)
		{
			queue_size = uxQueueMessagesWaiting(hqueue);
			
			update_MaxQueu_size(currentsize, uxQueueMessagesWaiting(hqueue));
		}
		sim_time = xTaskGetTickCount();
		sim_time_hours = (sim_time / 600) + 9;
		sim_time_mins = (sim_time / 10) % 60;
		//Customer_Maxupdate(customer);//update max time
		teller_1_state = displayTellerState(hteller_1);
		teller_2_state = displayTellerState(hteller_2);
		//teller_3_state = displayTellerState(hteller_3);
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
			int avg_customer_wait = total_wait_time / total_customers_served;
			int avg_teller_idle_time = (4200 - total_time_serving_customers) / 30;
			int max_queue_size =  currentsize.size;
			int max_TT_time = getMaxTT(customer);
			n = sprintf((char *)buffer, 
				"\n\rTotal customers served = %d \n\r Maximum Transation Time = %d mins \n\r Average customer wait time = %d mins\n\r Maximum Time Spent In Queue = %d mins \n\r" 
				"Average time customers spend with teller = %d mins \n\r Average time tellers wait for customers = %d mins \n\r Teller 1 served %d customers \n\r Teller 2 served %d customers" 
				"\n\r Teller 3 served %d customers\n\r",
					total_customers_served,
					max_TT_time,
					avg_customer_wait,
					max_wait_time,
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
