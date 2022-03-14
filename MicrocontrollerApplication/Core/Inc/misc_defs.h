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
	uint8_t str_debug     : 1;

	union
	{
		uint8_t sensor_contents;

		struct
		{
			uint8_t ecg_ready     : 1;
			uint8_t emg_ready     : 1;
			uint8_t force_ready   : 1;
			uint8_t accel_ready   : 1;
			uint8_t gyro_ready    : 1;
			uint8_t pkt_ready     : 1;
			uint8_t queue_full    : 1;
		};
	};

} flags_t;

/* The payload contains the
 * sensor values
 */
/* The payload contains the
 * sensor values
 */
typedef struct payload_t
{
	uint16_t ecg_s[32];     //heart sensor
	uint16_t emg_s[32];	   //emg sensor
	float    force_s;      //force sensor
	float    accelx_s; //accelerometer sensor
	float    accely_s; //accelerometer sensor
	float    accelz_s; //accelerometer sensor
	float    gyrox_s;  //gyro sensor
	float    gyroy_s;  //gyro sensor
	float    gyroz_s;  //gyro sensor

	uint8_t  payload_size; //size of payload
} __attribute__((packed)) payload_t;

/* The packet contains all
 * overhead and meta-data
 */
typedef struct packet_t
{
	uint32_t  start;
	uint16_t  packet_size;   //size of packet
	uint16_t  packet_num;    //Packet number
	uint8_t   state : 4;     //The current state of the microcontroller
	uint8_t   reserved : 4;  //reserved
	payload_t payload;	     //payload struct
	char      timestamp[24]; //time packet was sent
} __attribute__((packed)) packet_t;

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
