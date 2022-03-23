/*
 * state_machine.c
 *
 *  Created on: Nov. 27, 2021
 *      Author: Chase B
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "misc_defs.h"
#include "state_machine.h"
#include "main.h"

extern flags_t global_flags;

/***
 * typedef enum state_machine
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
	STOP_FAIL      = -4

} state_machine;
 *
 */

/**-------------------------------------------
 * Function: set_state
 *
 * Author: Chase B
 *
 * Description: Set the state of the machine
 * 				Make sure that the state changes
 * 				are allowed and execute necessary
 * 				code when changing states
 *
 * Parameters: state_machine*, state_machine
 *
 * Return Value: bool
 *-------------------------------------------*/
bool set_state(state_machine* curr_state, state_machine new_state)
{
	switch(*curr_state)
	{
	case NONE:
		if(new_state == STARTUP || new_state < 0)
		{
			*curr_state = new_state;
			return true;
		}
		else
		{
			return false;
		}
	case STARTUP:
		if(new_state == CONFIGURING || new_state < 0)
		{
			*curr_state = new_state;
			return true;
		}
		return false;

	case CONFIGURING:
		if(new_state == SEARCHING || new_state < 0)
		{
			*curr_state = new_state;
			return true;
		}
		return false;

	case SEARCHING:
		if(new_state == CONNECTING || new_state < 0)
		{
			*curr_state = new_state;
			return true;
		}
		return false;

	case CONNECTING:
		if(new_state == READY || new_state < 0)
		{
			*curr_state = new_state;
			return true;
		}
		return false;

	case READY:
		if(new_state == READING || new_state < 0)
		{
			*curr_state = new_state;
			return true;
		}
		return false;

	case READING:
		if(new_state == STORING && global_flags.pkt_ready == true || new_state < 0)
		{
			*curr_state = new_state;
			return true;
		}

	case STORING:
		if(new_state == READING || new_state < 0)
		{
			*curr_state = new_state;
			return true;
		}

	case SENDING:
		if(new_state == READING || new_state < 0)
		{
			*curr_state = new_state;
			return true;
		}
	default:
		return false;
	}

	return false;
}

void state_action(state_machine state)
{
	switch(state)
	{
	case NONE:
		return;

	case STARTUP:
		return;

	case CONFIGURING:
		return;

	case SEARCHING:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(250);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(2000);
		return;

	case CONNECTING:
		return;

	case READY:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_SET);
		HAL_Delay(750);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, GPIO_PIN_RESET);
		HAL_Delay(2000);
		return;

	case READING:
		return;

	case STORING:
		return;

	case SENDING:
		return;

	default:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, 0);
		return;

	}
}

