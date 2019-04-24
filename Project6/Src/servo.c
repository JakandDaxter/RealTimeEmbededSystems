#include "servo.h"

struct Servo* createServo(QueueHandle_t q, int channel)
{
	struct Servo* servo = (struct Servo*) malloc(sizeof(struct Servo));
	servo->position = 0;
	servo->state = state_idle;
	servo->status = status_paused;
  servo->q = q;
  servo->channel = channel;
	return servo;
}