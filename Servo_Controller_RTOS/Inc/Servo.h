#ifndef __SERVO_H
#define __SERVO_H

#include <stdio.h>
#include <stdlib.h>

#include "stm32l476xx.h"
#include "States.h"

struct Servo
{
	enum servo_states state;
	int position;
	enum status status;
	int id;
	int count;
};

void pause_servo(struct Servo* servo);
void start_servo(struct Servo *servo);
void move_servo_to(struct Servo* servo, int position);
void continue_servo(struct Servo *servo);
int executeNextInstruction(struct Servo* servo);
void move_left(struct Servo* servo);
void move_right(struct Servo* servo);
#endif /*__SERVO_H*/