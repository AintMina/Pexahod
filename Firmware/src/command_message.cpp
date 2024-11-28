#include "command_message.h"


#include <string.h>


void command_message_init(struct command_message_t *msg) {
    memset(msg, 0, sizeof(struct command_message_t));
    msg->prefix = PREFIX;
}

void command_create_message(struct command_message_t *msg, uint8_t length, commands_e command, uint8_t data[8]) {

    command_message_init(msg);
    msg->prefix = PREFIX;
    msg->length = length;
    msg->id = ID;
    msg->command = command;
    for (int i = 0; i < msg->length; i++) {
        msg->data[i] = data[i];
    }
    msg->crc = command_calculate_crc(msg);
    msg->data[msg->length] = msg->crc;
}

uint8_t command_calculate_crc(struct command_message_t *msg) {
    uint32_t sum = 0;
    uint8_t crc = 0;

    uint8_t* byte_ptr = (uint8_t*) msg;

    for (int i = 0; i < (sizeof(struct command_message_t) - sizeof(msg->data) + msg->length) - 1; i++) {
        sum += byte_ptr[i];
    }

    crc = 0x100 - (sum & 0xff);

    return crc;
}