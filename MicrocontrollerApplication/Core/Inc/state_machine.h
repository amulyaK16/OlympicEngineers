/*
 * state_machine.h
 *
 *  Created on: Nov. 15, 2021
 *      Author: chase
 */

#ifndef INC_STATE_MACHINE_H_
#define INC_STATE_MACHINE_H_

typedef enum state_machine
{
	NONE          = 0, //Unknown state
	STARTUP       = 1, //Initial Power on
	CONFIGURING   = 2, //Setting up MCU
	SEARCHING     = 3, //Searching for bluetooth
	CONNECTING    = 4, //Connecting to server
	READY         = 5, //Connected and ready to transmit
	READING       = 6, //Reading sensor values
	STORING		  = 7, //Storing sensor values
	SENDING       = 8, //Sending values to server
	PAUSED        = 9, //Paused
	POWERING_DOWN = 10, //Powering Down Device

	SENSOR_FAULT   = -1,
	SEARCH_TIMEOUT = -2,
	START_FAIL     = -3,
	STOP_FAIL      = -4,
	INIT_QUEUE_FAIL= -5

} state_machine;

bool set_state(state_machine* curr_state, state_machine new_state);

#endif /* INC_STATE_MACHINE_H_ */
