#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "misc_defs.h"
#include "queue.h"

/**
 * Chase Badalato
 *
 * A packet queue implemented
 * using a linked list
 *
**/

/**------------------------------------------
 * Function: create_queue
 *
 * Author: Chase B
 *
 * Description: Create a new queue struct and
 * 				initialize the front and back
 * 				pointer to NULL.
 *
 * Parameters:
 *
 * Return Value: packet_queue*
 -------------------------------------------*/
packet_queue* create_queue()
{
	packet_queue* new_queue = (packet_queue*)malloc(sizeof(packet_queue));
	new_queue->back = NULL;
	new_queue->front = NULL;
	new_queue->size = 0;

	return new_queue;
}

/**------------------------------------------
 * Function: add_node
 *
 * Author: Chase B
 *
 * Description: Create a new packet_node
 *
 * Parameters: packet_t
 *
 * Return Value: packet_node*
 -------------------------------------------*/
packet_node* add_node(packet_t packet_info)
{
	packet_node* new_node = (packet_node*)malloc(sizeof(packet_node));
	new_node->packet = packet_info;
	new_node->next_node = NULL;

	return new_node;
}

/**-------------------------------------------
 * Function: add_packet
 *
 * Author: Chase B
 *
 * Description: add a new packet to the queue
 *
 * Parameters: packet_queue*, packet_t
 *
 * Return Value: void
 *-------------------------------------------*/
uint8_t add_packet(packet_queue* queue, packet_t packet)
{
	if(queue->size >= MAX_QUEUE_SIZE)
	{
		return false;
	}
	else
	{
		packet_node* tmp_node = add_node(packet);
		if(tmp_node == NULL)
		{
			return false;
		}
		else
		{
			//Queue is empty
			if(queue->back == NULL)
			{
				queue->front = tmp_node;
				queue->back  = queue->front;
			}
			//Queue must have at least one value
			else
			{
				queue->back->next_node = tmp_node;
				queue->back = tmp_node;
			}

			queue->size ++;
			return true;
		}
	}
}

/**-------------------------------------------
 * Function: rem_packet
 *
 * Author: Chase B
 *
 * Description: Remove a packet, returning the
 * 				packet contents
 *
 * Parameters: packet_queue*
 *
 * Return Value: packet_t
 *-------------------------------------------*/
packet_t rem_packet(packet_queue* queue)
{
	packet_t pkt = {};

	if(queue->front == NULL)
	{
		return pkt;
	}

	packet_node* tmp_node = queue->front;
	queue->front = queue->front->next_node;

	if(queue->front == NULL)
	{
		queue->back = NULL;
	}

	pkt = tmp_node->packet;
	queue->size --;
	free(tmp_node);
	return pkt;

}


