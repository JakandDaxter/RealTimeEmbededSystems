/* Includes ------------------------------------------------------------------*/
#include "gyro_task.h"


/* Gyroscope variables */
float gyro_velocity[3] = {0};       // angular velocity
int32_t gyro_angle[3] = {0, 0, 0};	// angle
extern QueueHandle_t hservo1_q;

void gyro_task_init() {
  
  if(BSP_GYRO_Init() != HAL_OK)  {
    /* Initialization Error */
    Error_Handler();
  }
  if (pdPASS != xTaskCreate (gyro_task,	"print", 256, NULL, osPriorityNormal, NULL)) {
    Error_Handler();
  }
}

void gyro_task(void* argument) {
  char buf[100];
  int new_position = 0;
  while(1) {

		BSP_GYRO_GetXYZ(gyro_velocity);   // get raw values from gyro device
    
    // integrate angular velocity to get angle
    for(int ii=0; ii<3; ii++) 
    {
      //for(int i=0; i<SAMPLE; i++)
      //{
        gyro_angle[ii] += (int32_t)(gyro_velocity[ii] / GYRO_THRESHOLD_DETECTION);
      //}
    }
    new_position = (gyro_angle[2] + 30);
    sprintf(buf, "X = %d\tY = %d\tZ = %d\n\r",gyro_angle[0], gyro_angle[1], gyro_angle[2]);
    vPrintString(buf);
    osDelay(100);
  }
}
