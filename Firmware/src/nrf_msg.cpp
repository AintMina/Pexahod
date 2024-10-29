#include "nrf_msg.h"


#include <string.h>


void nrf_message_init(struct nrf_message_t *msg) {
    memset(msg, 0, sizeof(struct nrf_message_t));
    msg->prefix = PREFIX;
}

void nrf_create_message(struct nrf_message_t *msg, uint8_t length, commands_t command, uint8_t data[8]) {

    nrf_message_init(msg);
    msg->prefix = PREFIX;
    msg->length = length;
    msg->id = ID;
    msg->command = command;
    for (int i = 0; i < msg->length; i++) {
        msg->data[i] = data[i];
    }
    msg->crc = nrf_calculate_crc(msg);
    msg->data[msg->length] = msg->crc;
}

uint8_t nrf_calculate_crc(struct nrf_message_t *msg) {
    uint32_t sum = 0;
    uint8_t crc = 0;

    uint8_t* byte_ptr = (uint8_t*) msg;

    for (int i = 0; i < (sizeof(struct nrf_message_t) - sizeof(msg->data) + msg->length) - 1; i++) {
        sum += byte_ptr[i];
    }

    crc = 0x100 - (sum & 0xff);

    return crc;
}