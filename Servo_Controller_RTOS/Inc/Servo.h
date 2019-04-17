#ifndef __SERVO_H
#define __SERVO_H

#include <stdlib.h>
#include "States.h"
#include "FreeRTOS.h"
#include "queue.h"

struct Servo
{
	enum servo_states state;
	int position;
	enum status status;
  QueueHandle_t q;
  int channel;
};

struct Servo* createServo(QueueHandle_t q, int channel);

#endif /*__SERVO_H*/