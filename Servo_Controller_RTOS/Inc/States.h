#ifndef __STATES_H
#define __STATES_H

#define MOV (0x20)
#define WAIT (0x40)
#define LOOP (0x80)
#define END_LOOP (0xA0)
#define RECIPIE_END (0)

// This is a good way to define the status of the display.
// This should be in a header (.h) file.
enum status 
{
	status_running,
	status_paused,
	status_command_error,
	status_nested_error
} ;

// This is a good way to define the state of a servo motor.
// This should be in a header (.h) file.
enum servo_states
{
	state_idle,
	state_moving
} ;

#endif
