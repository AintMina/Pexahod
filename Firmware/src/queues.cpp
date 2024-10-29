#include "queues.h"
#include "nrf_msg.h"


QueueHandle_t xProtocolQueues[MAX_QUEUES] = {NULL};

void init_queue(void) {
    for (int i = 0; i < MAX_QUEUES; i++) {
        xProtocolQueues[i] = xQueueCreate(10, sizeof(struct nrf_message_t));  // Create queue for each index
        if (xProtocolQueues[i] == NULL) {
            // Handle queue creation failure (e.g., log an error or trap CPU)
            while (1);  // Trap the CPU in case of failure (you can replace with logging)
        }
    }
}

BaseType_t send_to_queue(int index, const struct nrf_message_t *message, TickType_t xTicksToWait) {
    if (index >= 0 && index < MAX_QUEUES && xProtocolQueues[index] != NULL) {
        return xQueueSend(xProtocolQueues[index], message, xTicksToWait);  // Send to the specified queue
    }
    return pdFAIL;
}

BaseType_t receive_from_queue(int index, struct nrf_message_t *message, TickType_t xTicksToWait) {
    if (index >= 0 && index < MAX_QUEUES && xProtocolQueues[index] != NULL) {
        return xQueueReceive(xProtocolQueues[index], message, xTicksToWait);  // Receive from the specified queue
    }
    return pdFAIL;
}