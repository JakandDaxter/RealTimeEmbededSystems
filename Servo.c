#include "Servo.h"

#define PARAM 0x1F
#define OP 0xE0
#define MIN 0
#define MAX 5

struct Servo* createServo(uint8_t* commands, int size)
{
	struct Servo* servo = (struct Servo*) malloc(sizeof(struct Servo));
	servo->callStack = createServoCallStack(commands, size);
	servo->position = 0;
	servo->state = state_idle;
	servo->status = status_paused;
	return servo;
}

void start_servo(struct Servo *servo)
{
	servo->callStack->top = 0;
	servo->status = status_running;
}

void continue_servo(struct Servo *servo)
{
	if((servo->status == status_nested_error) || servo->status == status_command_error) return; 
	servo->callStack->top = servo->callStack->pausePointer;
	servo->status = status_running;
}

void pause_servo(struct Servo* servo)
{
	if(servo->state != state_moving)
	{
		servo->callStack->pausePointer = servo->callStack->top;
		servo->callStack->top = servo->callStack->capacity;
		servo->status = status_paused;
		servo->state = state_idle;
	}
}

void move_servo_to(struct Servo* servo, int position)
{
	pause_servo(servo);
	move_to_position(servo,position);
	continue_servo(servo);
	return;
}

int executeNextInstruction(struct Servo* servo)
{
	uint8_t instruction = pop(servo->callStack);
	uint8_t op = instruction & OP;
	uint8_t parameter = instruction & PARAM;
	if(instruction == 0xFF)
	{
		return -1;
	}
	else
	{
		switch(op)
		{
			case MOV:
				if(parameter > MAX)
				{
					servo->status = status_command_error;
					return -1;
				}
				move_servo_to(servo,parameter);
				break;
			case WAIT:
				if((parameter > 0) && (servo->callStack->loopCount < 0)){
					servo->callStack->top--;
					servo->callStack->loopCount = parameter - 1;
					if(servo->state != state_idle)
					{
						servo->state = state_idle;
					}
					return 0;
				}
				else if(servo->callStack->loopCount > 0){
					servo->callStack->top--;
					servo->callStack->loopCount--;
					return 0;
				}
				else if(parameter == 0)
				{
					return 0;
				}
				servo->callStack->loopCount--;
				return 0;
			case LOOP:
				servo->callStack->depth++;
				servo->callStack->loopPointer = servo->callStack->top;
				servo->callStack->loopCount = parameter;
				return 0;
			case END_LOOP:
				if(servo->callStack->depth > 1)
				{
					servo->status = status_nested_error;
					return -1;
				} 
				else if(servo->callStack->loopCount >= 0)
				{
					servo->callStack->depth--;
					servo->callStack->top = servo->callStack->loopPointer;
					servo->callStack->loopCount--;
				}
				return 0;
			case RECIPE_END:
				Print("End has been reached");
				return 0;
			default: servo->status = status_command_error;
		}
		return 0;
	}
}

