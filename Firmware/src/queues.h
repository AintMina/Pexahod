#ifndef QUEUES_H
#define QUEUES_H



#include <FreeRTOS.h>
#include <queue.h>

#define MAX_DATA_SIZE 64
#define MAX_QUEUES 2


struct command_message_t;

void init_queue(void);
BaseType_t send_to_queue(int index, const struct command_message_t *message, TickType_t xTicksToWait);
BaseType_t receive_from_queue(int index, struct command_message_t *message, TickType_t xTicksToWait);



#endif // QUEUES_H