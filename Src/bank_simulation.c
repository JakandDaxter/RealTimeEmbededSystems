#include "bank_simulation.h"
#define BUFFER_SIZE 128
#define UART_TIMEOUT 100000

extern UART_HandleTypeDef huart2;
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
	uint32_t n,k,f,g; //shit ton cause i just wanted it to work
	uint32_t       Customer_Arrival;
	uint32_t         Customer_TT; //transation time
	uint8_t buffer[BUFFER_SIZE];
	HAL_UART_Transmit(&huart2,(uint8_t*)"Customers Arriving\r\n", 20, UART_TIMEOUT);
	BaseType_t xStatus;
    for(;;)
	{
        
        f = (RNG ->DR);
        Customer_TT = (f)%751 + 50;
        Customer_Arrival = (f)%301 + 100;
        
		xStatus = xQueueSendToBack(hqueue, &Customer_TT, 0);
		if(xStatus == pdPASS)
		{
			
			
		  // printf("Customer_ %d, Transation Time = %d\n", i , Customer_TT );
			g = sprintf((char *)buffer, "%s", "Customer_");
			HAL_UART_Transmit(&huart2, buffer, g, UART_TIMEOUT);
				
			k = sprintf((char *)buffer, "%d\t", Customer_Arrival);
			HAL_UART_Transmit(&huart2, buffer, k, UART_TIMEOUT);
		   // printf(number);
			n = sprintf((char *)buffer, "%d\n\n\r",Customer_TT);
			HAL_UART_Transmit(&huart2, buffer, n, UART_TIMEOUT);
		}
	    vTaskDelay(Customer_Arrival);
	}
}

void vTeller(void *pvParameters)
{
	uint32_t n;
	uint32_t customer_tt;
	uint8_t buffer[BUFFER_SIZE];
	for(;;)
	{
		if(hqueue != 0)
		{
			if(xQueueReceive(hqueue, &customer_tt, 10) == pdPASS)
			{
				n = sprintf((char *)buffer, "Teller busy for %d ticks \n\r", customer_tt);
				HAL_UART_Transmit(&huart2, buffer, n, UART_TIMEOUT);
			}
		}
		vTaskDelay(customer_tt);
	}
}
