/*
 * sensor_defs.h
 *
 *  Created on: Nov. 15, 2021
 *      Author: chase
 */

#ifndef INC_MISC_DEFS_H_
#define INC_MISC_DEFS_H_

#define ANALOG_DMA_SIZE 200

//Create a boolean data value
typedef enum
{
	true,
	false
} bool;

typedef struct flags_t
{
	uint8_t ble_interrupt : 1; //Bluetooth Interrupt
	uint8_t can_transmit  : 1; //
	uint8_t analog_hf     : 1; //Analog buffer half full
	uint8_t analog_ff     : 1; //Full full

	uint8_t ecg_ready     : 1;
	uint8_t emg_ready     : 1;
	uint8_t force_ready   : 1;
	uint8_t accel_ready   : 1;
	uint8_t pkt_ready     : 1;
	uint8_t queue_full    : 1;

} flags_t;

/* The payload contains the
 * sensor values
 */
typedef struct payload_t
{
	uint16_t heart_s;      //heart sensor
	uint16_t emg_s[50];	   //emg sensor
	uint16_t force_s[50];  //force sensor
	uint16_t accel_s;      //accelerometer sensor
	uint8_t  payload_size; //size of payload
} payload_t;

/* The packet contains all
 * overhead and meta-data
 */
typedef struct packet_t
{
	uint8_t   state : 4;    //The current state of the microcontroller
	uint8_t   reserved : 4; //reserved
	payload_t payload;	    //payload struct
	uint16_t  timestamp;    //time packet was sent
	uint8_t   packet_size;  //size of packet

} packet_t;

/* A single node of the queue */
typedef struct packet_node
{
	packet_t packet;
	struct packet_node* next_node;

} packet_node;

/* Structure that contains pointer
 * to the actual packet queue
 */
typedef struct packet_queue
{
	packet_node* front;
	packet_node* back;
	uint16_t     size;
	uint8_t      full;

} packet_queue;

#endif /* INC_MISC_DEFS_H_ */
