#include "command_handler.h"
#include "queues.h"
#include "command_message.h"
#include "errors.h"
#include "led.h"
#include "servos.h"
#include "robot.h"

#include <string.h>


const uint16_t command_delay = 1;

void command_handler_main(void *pvParameters) {
    while (1) {

        struct command_message_t message;
        BaseType_t ret = receive_from_queue(0, &message, 1);

        if (ret == pdPASS) {
            // Check prefix
            if (message.prefix != PREFIX) {
                continue;
            }

            // Relocate CRC
            message.crc = message.data[message.length];
            message.data[message.length] = 0;

            // Checking CRC
            uint8_t crc = command_calculate_crc(&message);
#if 1
            if (crc != message.crc) {
                uint8_t data[1] = {ERR_CRC};
                command_create_message(&message, 1, COMMAND_ERROR, data);
                send_to_queue(1, &message, 1);
            }
#endif

            switch (message.command) {
                case COMMAND_ERROR: {
                    /* code */
                    break;
                }
                case LEFT_J_X: {
                    /* code */
                    uint32_t combined = (message.data[3] << 24) | (message.data[2] << 16) | (message.data[1] << 8) | message.data[0];
                    float value = 0;
                    memcpy(&value, &combined, sizeof(value));
                    set_x_velocity(value);
                    break;
                }
                case LEFT_J_Y: {
                    /* code */
                    uint32_t combined = (message.data[3] << 24) | (message.data[2] << 16) | (message.data[1] << 8) | message.data[0];
                    float value = 0;
                    memcpy(&value, &combined, sizeof(value));
                    set_y_velocity(value);
                    break;
                }
                case LEFT_J: {
                    /* code */
                    break;
                }
                case RIGHT_J_X: {
                    /* code */
                    uint32_t combined = (message.data[3] << 24) | (message.data[2] << 16) | (message.data[1] << 8) | message.data[0];
                    float value = 0;
                    memcpy(&value, &combined, sizeof(value));
                    set_rotation_velocity(value);
                    break;
                }
                case RIGHT_J_Y: {
                    /* code */
                    break;
                }
                case RIGHT_J: {
                    /* code */
                    break;
                }
                case LEFT_SHOULDER: {
                    /* code */
                    break;
                }
                case RIGHT_SHOULDER: {
                    /* code */
                    break;
                }
                case BUTTONS: {
                    /* code */
                    break;
                }
                case VOLTAGE: {
                    /* code */
                    break;
                }
                case CURRENT: {
                    /* code */
                    break;
                }
                case LEDS: {
                    set_led(message.data[0], message.data[1], message.data[2], message.data[3]);
                    break;
                }
                case SET_SERVO: {
                    uint32_t intRepresentation = 0;
                    for (size_t i = 0; i < 4; i++) {
                        intRepresentation |= ((uint32_t)message.data[i+1]) << (i * 8);
                    }

                    float angle = *((float*)&intRepresentation);
                    set_servo(message.data[0], angle);

                    break;
                }
                case SET_LEG_SERVO: {
                    /* code */
                    break;
                }
                
                default: {
                    break;
                }
            }
        }

        vTaskDelay(command_delay / portTICK_PERIOD_MS);
    }
}