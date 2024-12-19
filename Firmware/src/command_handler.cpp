#include "command_handler.h"
#include "queues.h"
#include "command_message.h"
#include "errors.h"
#include "led.h"
#include "servos.h"
#include "robot.h"
#include "leg.hpp"
#include "unit_conversion.h"

#include <string.h>


const uint16_t command_delay = 10;

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
#if 0
            if (crc != message.crc) {
                uint8_t data[1] = {ERR_CRC};
                command_create_message(&message, 1, COMMAND_ERROR, data);
                send_to_queue(USB_QUEUE, &message, 1);
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
                case SET_MODE: {
                    set_mode((enum mode_e)message.data[0]);
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
                case SET_LEG_SERVO_OFFSETS: {
                    uint8_t leg_nr = message.data[0] - 1;
                    float offsets[3];

                    all_legs[leg_nr]->get_servo_offsets(offsets);

                    if (leg_nr < 6 && message.length == 13) {
                        for (int i = 0; i < 3; i++) {
                            offsets[i] =+ uint_to_float(&message.data[(i * sizeof(float)) + 1]);
                        }
                        all_legs[leg_nr]->set_servo_offsets(offsets);
                    }
                    break;
                }
                case GET_LEG_SERVO_SETTINGS: {
                    // Define an array of robot_settings_t structures to hold servo settings for 6 legs.
                    struct robot_settings_t settings;

                    // Extract the leg number from the incoming message data.
                    uint8_t leg_nr = message.data[0] - 1;

                    // Ensuring that the leg nr is between 0 and 5.
                    if (leg_nr < 6) {
                        read_robot_settings(&settings); // Read settings into the 'settings' array.

                        // Prepare an array to hold the serialized data for 3 float values (servo offsets).
                        uint8_t data[3*sizeof(float)];

                        // Iterate through the 3 servo offsets for the specified leg.
                        for (int i = 0; i < 3; i++) {
                            // Retrieve the servo offset value for the current servo.
                            float settings_value = settings.servo_offsets[leg_nr][i];
                            // Serialize the float value into a temporary uint8_t array.
                            uint8_t settings_value_data[sizeof(float)];
                            float_to_uint(settings_value, settings_value_data);

                            // Copy the serialized float data into the appropriate position in the 'data' array.
                            for (int ii = 0; ii < sizeof(float); ii++) {
                                data[(sizeof(float)*i) + ii] = settings_value_data[ii];
                            }
                        }

                        // Create a command message to send the serialized data back.
                        struct command_message_t answer;
                        answer.length = sizeof(float) * 3; // Set the length to accommodate 3 floats.
                        
                        // Populate the command message with the serialized data and relevant metadata.
                        command_create_message(&answer, answer.length, GET_LEG_SERVO_SETTINGS, data);

                        // Send the command message to the queue for transmission.
                        send_to_queue(USB_QUEUE, &answer, 1);
                    }
                    break;
                }
                case SAVE_SERVO_SETTINGS: {
                    /* code */
                    save_robot_settings();
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