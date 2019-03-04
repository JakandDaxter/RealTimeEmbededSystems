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
	servo->count = 1;
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

void stop_servo(struct Servo* servo)
{
	if(servo->state != state_moving)
	{
		servo->callStack->top = servo->callStack->capacity;
		servo->status = status_paused;
		servo->callStack->pausePointer = 0;
		servo->state = state_idle;
	}
}

void move_right(struct Servo* servo)
{
	if((servo->status == status_paused) && (servo->position > 0))
	{
		move_to_position(servo,servo->position - 1);
		servo->position--;
	}
	else if(servo->position == 0)
	{
		move_to_position(servo,servo->position);
	}
}
	
void move_left(struct Servo* servo)
{
	if((servo->status == status_paused) && (servo->position < 5))
	{
		move_to_position(servo,servo->position + 1);
		servo->position++;
	}
	else if(servo->position == 5)
	{
		move_to_position(servo,servo->position);
	}
}

int executeNextInstruction(struct Servo* servo)
{
	indicate_status();
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
				int count = parameter - servo->position;
				servo->count = servo->count ^ 1;
				if(count > 0)
				{
					move_to_position(servo,parameter);
					servo->position = servo->position + servo->count;
					servo->callStack->top--;
					return 0;
				}
				else if(count < 0)
				{
					move_to_position(servo,parameter);
					servo->position = servo->position - servo->count;
					servo->callStack->top--;
					return 0;
				} 
				else
				{
					move_to_position(servo,parameter);
					return 0;
				}
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
			case RECIPIE_END:
				stop_servo(servo);
				return 0;
			default: servo->status = status_command_error;
		}
		return 0;
	}
}

