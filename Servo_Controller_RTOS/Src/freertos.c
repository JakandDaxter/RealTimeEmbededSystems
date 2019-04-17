/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "servo.h"
#include "main.h"
#include "cmsis_os.h"
#include <string.h>
#include <stdio.h>
#include <ctype.h>
#include "instruction_set.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define NEXT_CLOCK  100 //next time an instruction will be called 
#define BUFFER_SIZE 128
#define UART_TIMEOUT 1000
#define NUM_SERVOS 2
#define PARAM 0x1F
#define OP 0xE0
#define MIN 0
#define MAX 5
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
QueueHandle_t hservo1_q;
QueueHandle_t hservo2_q;

TaskHandle_t servo1_instructions;
TaskHandle_t servo2_instructions;

struct Servo* s1;
struct Servo* s2;

uint8_t commands[20] = {MOV+0,MOV+5,MOV+0,MOV+3,LOOP+1,MOV+1,MOV+4,END_LOOP,MOV+0,MOV+2,WAIT+0,MOV+3,WAIT+0,MOV+2,MOV+3,WAIT+31,WAIT+31,WAIT+31,MOV+0,RECIPIE_END};
uint8_t commands2[7] = {MOV+0,MOV+1,MOV+2,MOV+3,MOV+4,MOV+5, RECIPIE_END};
uint8_t test_move_to_all_posistions_commands[6] = {MOV+0,MOV+1,MOV+2,MOV+3,MOV+4,MOV+5};
uint8_t test_nested_loop_error_commands[8] = {MOV+0, MOV+1, LOOP+2, LOOP+3, MOV+0, END_LOOP, MOV+2, END_LOOP};
uint8_t test_end_recipie_and_continue_commands[3] = {MOV+3, RECIPIE_END, MOV+5};
uint8_t test_command_error_commands[3] = {MOV+0, MOV+7, MOV+5};

extern UART_HandleTypeDef huart2;

extern TIM_HandleTypeDef htim2;

extern uint8_t aTxStartMessage;

uint8_t rx_index;
extern uint8_t rx_byte;

struct Instruction_Set * iset1 = NULL;
struct Instruction_Set * iset2 = NULL;

SemaphoreHandle_t command_mutex = NULL;
SemaphoreHandle_t receive_mutex = NULL;
SemaphoreHandle_t transmit_mutex = NULL;
SemaphoreHandle_t iset_mutex = NULL;
  
/* Buffer used for reception */
extern uint8_t aRxBuffer[20];
extern uint8_t ucOxBuffer[NUM_SERVOS];
/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void green_led_toggle(void);
void red_led_toggle(void);
void vTask2( void *pvParameters );
void vTask1( void *pvParameters );
void vServo(void *pvParameters);
void vUI(void *pvParameters);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

extern void MX_USB_HOST_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */     
  /* USER CODE END Init */
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	command_mutex = xSemaphoreCreateMutex();
	receive_mutex = xSemaphoreCreateMutex();
	transmit_mutex = xSemaphoreCreateMutex();
  iset_mutex = xSemaphoreCreateMutex();
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */
  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);
  
  /* USER CODE BEGIN RTOS_QUEUES */
  hservo1_q = xQueueCreate(100, sizeof(uint8_t));
  hservo2_q = xQueueCreate(100, sizeof(uint8_t));
  
  iset1 = createInstructionSet(hservo1_q, commands, servo1_instructions, 20);
 
	
	s1 = createServo(hservo1_q, TIM_CHANNEL_1);
	s2 = createServo(hservo2_q, TIM_CHANNEL_2);

  /* USER CODE BEGIN RTOS_THREADS */
  xTaskCreate( vTask1, "Task 1", 200, (void *)iset1, 1, NULL ); 
  iset2 = createInstructionSet(hservo2_q, commands2, servo2_instructions, 7);/* USER CODE END RTOS_QUEUES */
  xTaskCreate( vTask1, "Task 2", 200, (void *)iset2, 1, NULL );
  xTaskCreate( vServo, "Servo 1", 100, (void *)s1, 1, &servo1_instructions );
  xTaskCreate( vServo, "Servo 2", 100, (void *)s2, 1, &servo2_instructions );
  xTaskCreate( vUI, "User Interface", 100, NULL, 1, NULL);
  /* USER CODE END RTOS_THREADS */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void green_led_toggle()
{
	GPIOE->ODR ^= GPIO_ODR_ODR_8;
}
void red_led_toggle()
{
	GPIOB->ODR ^= GPIO_ODR_ODR_2;
}

void red_led_on(void)
{
	GPIOB->ODR |= GPIO_ODR_ODR_2;
}

void green_led_on(void)
{
	GPIOE->ODR |= GPIO_ODR_ODR_8;
}

void vPrintString(char *message)
{
	xSemaphoreTake(transmit_mutex, ~0);
	HAL_UART_Transmit_IT(&huart2, (uint8_t*)message, strlen(message));
	xSemaphoreGive(transmit_mutex);
}

void vUI(void *pvParameters)
{ 
	uint8_t i_next_command = 0;
	for(;;)
	{
			if((char)rx_byte == '\r')
			{
				switch(ucOxBuffer[0])
				{
					case 'B':
						if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
						{
							ucOxBuffer[0] = NULL; //clear receive buffer
							iset1->index = 0;
							xSemaphoreGive(command_mutex);
						}
						break;
					case 'P':
						if(servo1_instructions != NULL)
						{
							if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
							{
								ucOxBuffer[0] = NULL; //clear receive buffer
								i_next_command = iset1->index;
								iset1->index = iset1->len;
								xSemaphoreGive(command_mutex);
							}
						}
						break;
					case 'C':
						if(servo1_instructions != NULL)
						{
							if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
							{
								ucOxBuffer[0] = NULL; //clear receive buffer
								iset1->index = i_next_command;
								xSemaphoreGive(command_mutex);
							}
						}
						break;
					case 'N':
						ucOxBuffer[0] = NULL; //clear receive buffer
						break;
					case 'R':
						if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
						{
								ucOxBuffer[0] = NULL; //clear receive buffer
								if(iset1->index >= iset1->len)
								{
									if(s1->position > 0)
									{
										int np = s1->position - 1;
										xQueueSendToFront(s1->q, &np, (TickType_t) 100);
									}
								}
								xSemaphoreGive(command_mutex);
						}
						break;
					case 'L':
						if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
						{
							ucOxBuffer[0] = NULL; //clear receive buffer
							if(iset1->index >= iset1->len)
							{
								if(s1->position < 5)
								{
									int np = s1->position + 1;
									xQueueSendToFront(s1->q, &np, (TickType_t) 100);
								}
							}
							xSemaphoreGive(command_mutex);
						}
						break;
				}
//*****************************************************************************SERVO 2********************************************************************				
				switch(ucOxBuffer[1])
				{
					case 'B':
						if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
						{
							ucOxBuffer[1] = NULL; //clear receive buffer
							iset2->index = 0;
							xSemaphoreGive(command_mutex);
						}
						break;
					case 'P':
						if(servo1_instructions != NULL)
						{
							if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
							{
								ucOxBuffer[1] = NULL; //clear receive buffer
								i_next_command = iset2->index;
								iset2->index = iset2->len;
								xSemaphoreGive(command_mutex);
							}
						}
						break;
					case 'C':
						if(servo1_instructions != NULL)
						{
							if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
							{
								ucOxBuffer[1] = NULL; //clear receive buffer
								iset2->index = i_next_command;
								xSemaphoreGive(command_mutex);
							}
						}
						break;
					case 'N':
						ucOxBuffer[1] = NULL; //clear receive buffer
						break;
					case 'R':
						if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
						{
								ucOxBuffer[1] = NULL; //clear receive buffer
								if(iset2->index >= iset2->len)
								{
									if(s2->position > 0)
									{
										int np = s2->position - 1;
										xQueueSendToFront(s2->q, &np, (TickType_t) 100);
									}
								}
								xSemaphoreGive(command_mutex);
						}
						break;
					case 'L':
						if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
						{
							ucOxBuffer[1] = NULL; //clear receive buffer
							if(iset2->index >= iset2->len)
							{
								if(s2->position < 5)
								{
									int np = s2->position + 1;
									xQueueSendToFront(s2->q, &np, (TickType_t) 100);
								}
							}
							xSemaphoreGive(command_mutex);
						}
						break;
				}
			}
	}
}

void vTask1( void *pvParameters )
{
	struct Instruction_Set* iset = (struct Instruction_Set *)pvParameters;
	uint8_t new_position = 0; 
  uint8_t *commands = iset->commands;
	uint8_t delay_offset = 1;
	uint8_t operation = 1;
	uint8_t operand = 0;
	uint8_t loop_ptr = 0, loop_count = 0;
	for( ;; )
	{
		//green_led_toggle();
    if( xSemaphoreTake( iset_mutex, ( TickType_t ) 10 ) == pdTRUE )
    {
      operation = iset->commands[iset->index] & OP;
      xSemaphoreGive(iset_mutex);
    }
		if(operation != RECIPIE_END)
		{   
        switch(operation)
        {
        case MOV:
          new_position = operand;
          xQueueSendToBack(iset->hq, &new_position, (TickType_t) 10);
          break;
        case WAIT:
          delay_offset = operand;
          break;
        case LOOP:
          loop_ptr = iset->index;
          loop_count = operand;
          break;
        case END_LOOP:
          if(loop_count > 0)
          {
            loop_count--;
            if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
            {
              iset->index = loop_ptr;
              xSemaphoreGive(command_mutex);
            }
          }
          break;
        default:
          break;
        }
        xSemaphoreGive(iset_mutex);
    }
			if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
			{
				iset->index++;
				xSemaphoreGive(command_mutex);
			}
			vTaskDelay(NEXT_CLOCK * delay_offset);
		}
	}



  
  void vTask2( void *pvParameters )
{
	struct Instruction_Set* iset = (struct Instruction_Set *)pvParameters;
	uint8_t new_position = 0; 
  uint8_t *commands = iset->commands;
	uint8_t delay_offset = 1;
	uint8_t operation = 1;
	uint8_t operand = 0;
	uint8_t loop_ptr = 0, loop_count = 0;
	for( ;; )
	{
		//green_led_toggle();
    if( xSemaphoreTake( iset_mutex, ( TickType_t ) 10 ) == pdTRUE )
    {
      operation = iset->commands[iset->index] & OP;
      xSemaphoreGive(iset_mutex);
    }
		if(operation != RECIPIE_END)
		{   
        switch(operation)
        {
        case MOV:
          new_position = operand;
          xQueueSendToBack(iset->hq, &new_position, (TickType_t) 10);
          break;
        case WAIT:
          delay_offset = operand;
          break;
        case LOOP:
          loop_ptr = iset->index;
          loop_count = operand;
          break;
        case END_LOOP:
          if(loop_count > 0)
          {
            loop_count--;
            if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
            {
              iset->index = loop_ptr;
              xSemaphoreGive(command_mutex);
            }
          }
          break;
        default:
          break;
        }
        xSemaphoreGive(iset_mutex);
    }
			if( xSemaphoreTake( command_mutex, ( TickType_t ) 10 ) == pdTRUE )
			{
				iset->index++;
				xSemaphoreGive(command_mutex);
			}
			vTaskDelay(NEXT_CLOCK * delay_offset);
		}
}

void vServo(void *pvParameters)
{
  struct Servo* servo = (struct Servo*)pvParameters;
  int n = 0;
  uint8_t buffer[BUFFER_SIZE];
  uint8_t new_position = 0;
  int delay_offset = 0;
  for(;;)
  {
		if(xQueueReceive(servo->q, &new_position, (TickType_t) 10) == pdPASS)
		{
			delay_offset = new_position - servo->position;
			if(delay_offset < 0)
			{
				delay_offset = delay_offset*(-1);
			}
			servo->position = new_position;
			__HAL_TIM_SET_COMPARE(&htim2, servo->channel, 5 + (new_position * 3));
			vTaskDelay(2 * NEXT_CLOCK * delay_offset);
		}
  }
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
