#ifndef NRF_MSG_H
#define NRF_MSG_H

#include <stdint.h>


#define PREFIX 0x69
#define ID 0x00

enum commands_t : uint8_t {
    COMMAND_ERROR,
    LEFT_J_X,
    LEFT_J_Y,
    LEFT_J,
    RIGHT_J_X,
    RIGHT_J_Y,
    RIGHT_J,
    LEFT_SHOULDER,
    RIGHT_SHOULDER,
    BUTTONS,
    VOLTAGE,
    CURRENT,
    LEDS,
    SET_SERVO,
    SET_LEG_SERVO,
    COMMAND_ENUM_SIZE,
};

struct nrf_message_t {
    uint8_t prefix;
    uint8_t length;
    uint8_t id;
    uint8_t command;
    uint8_t data[32];
    uint8_t crc;
}__attribute__((packed));

struct joystick_message_t {
    uint8_t id;
    uint8_t prefix;
    uint8_t x_left;
    uint8_t y_left;
    uint8_t x_right;
    uint8_t y_right;
    uint8_t buttons;
    uint8_t shoulder_left;
    uint8_t shoulder_right;
    uint8_t gyro_x;
    uint8_t gyro_y;
    uint8_t gyro_z;
    uint8_t acc_x;
    uint8_t acc_y;
    uint8_t acc_z;
} __attribute__((packed));

void nrf_message_init(struct nrf_message_t *msg);
void nrf_create_message(struct nrf_message_t *msg, uint8_t length, commands_t command, uint8_t data[8]);
uint8_t nrf_calculate_crc(struct nrf_message_t *msg);

#endif