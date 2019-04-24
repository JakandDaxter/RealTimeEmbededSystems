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
#include "main.h"
#include "gyro_task.h"
#include "Servo.h"
#include "cmsis_os.h"
#include "basic_io.h"

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
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
extern TIM_HandleTypeDef htim1;
extern UART_HandleTypeDef huart2;
uint32_t discCCR = 500;
uint32_t contCCR = 500; 
uint16_t discreteDelay = 5000;

/* USER CODE END Variables */
osThreadId defaultTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void vDiscreteServo(void *pvParameters);
void vContinousServo(void *pvParameters);
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);

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
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  BSP_GYRO_Init();
  xTaskCreate( vContinousServo, "Servo 1", 100, NULL, 1, NULL );
  xTaskCreate( vDiscreteServo, "Servo 2", 100, NULL, 1, NULL );
  /* USER CODE END RTOS_QUEUES */
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

void vContinousServo(void *pvParameters)
{
	int new_position = 0;
	float gyro_velocity[3] = {0};       // angular velocity
	int32_t gyro_angle[3] = {0, 0, 0};	// angle
  int offset = 0;
	for(;;)
	{
		BSP_GYRO_GetXYZ(gyro_velocity);   // get raw values from gyro device
		// integrate angular velocity to get angle
		for(int ii=0; ii<3; ii++) 
		{
			gyro_angle[ii] += (int32_t)(gyro_velocity[ii] / GYRO_THRESHOLD_DETECTION);
		}
		new_position = -1 * gyro_angle[2];
		if(new_position < 0)
		{
			new_position = 0;
		}
		else if(new_position > 1500)
		{
			new_position = 1500;
		}
    contCCR = 500 + new_position;
		__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, contCCR);
		vTaskDelay(100);
	}
}	


void vDiscreteServo(void *pvParameters)
{
	int new_position = 0;
	int delay_offset = 0;
  char cBuffer[128];
	uint8_t p, hits = 0, misses = 0;
  int j = 0;
	for(;;)
	{
    while( j < 11)
    {
      uint32_t min_angle = discCCR * 0.95;
      uint32_t max_angle = discCCR * 1.05;
      if((contCCR > min_angle) && (contCCR < max_angle))
      {
        GPIOE->ODR |= 0x1<<8;
        hits++;
      }
      else
      {
        GPIOE->ODR &= ~(0x1<<8);
        misses++;
      }
      p = (RNG->DR);
      while(p%6 == new_position)
      {
        p = (RNG->DR);
      }
      new_position = p%6;
      if(delay_offset < 0)
      {
        delay_offset = delay_offset*(-1);
      }
      discCCR = 500 + (new_position * 300);
      j++;
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, discCCR);
      vTaskDelay(discreteDelay);
    }
    int n = sprintf(cBuffer, "GAME OVER   HITS = %d   MISSES = %d\r", hits - 1, misses);
    HAL_UART_Transmit(&huart2 ,(uint8_t*)cBuffer , n, 1000);
    vTaskDelay(100);
	}
}
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
