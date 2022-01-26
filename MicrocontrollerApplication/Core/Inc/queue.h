/*
 * queue.h
 *
 *  Created on: Nov. 15, 2021
 *      Author: chase
 */

#ifndef INC_QUEUE_H_
#define INC_QUEUE_H_

#define MAX_QUEUE_SIZE 8

typedef struct payload_t payload_t;
typedef struct packet_t packet_t;
typedef struct packet_queue packet_queue;
typedef struct packet_node packet_node;

/**
 * Queue Structure:
 * Each older node points to the newly inserted node
 *
 * | BACK | <-- | NODE | <-- | NODE | <-- | NODE | <-- | FRONT |
 *
 */

packet_queue* create_queue();
packet_node* add_node(packet_t packet_info);
uint8_t add_packet(packet_queue* queue, packet_t packet);
packet_t rem_packet(packet_queue* queue);

#endif /* INC_QUEUE_H_ */
