/*

     (1)               (6)
       \               /
        \             /
         \           /
          *---(F)---*
         /           \
        /             \
       /               \
(2)---*                 *---(5)
       \               /
        \             /
         \           /
          *---(B)---*
         /           \
        /             \
       /               \
     (3)               (4)

*/

// System
#include "pico/stdlib.h"
#include <math.h>

// Servo2040
#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"


// Local
#include "robot_controller.hpp"
#include "led.h"
#include "nrf.h"
#include "queues.h"
#include "usb.h"
#include "nrf_msg.h"
#include "command_handler.h"
#include "sensors.h"

using namespace servo;

TaskHandle_t xDumbModeHandle = NULL;
TaskHandle_t xNrfHandle = NULL;
TaskHandle_t xUsbHandle = NULL;
TaskHandle_t xCommandHandle = NULL;
TaskHandle_t xSensorHandle = NULL;

TaskHandle_t xTempHandle = NULL;


int main() {
    // NEEDED FOR DEBUG!!!
    timer_hw->dbgpause = 0;

    stdio_init_all();

    stdio_set_translate_crlf(&stdio_usb, false);
    sleep_ms(10);

    init_queue();
    

	led_init();
    for (int i = 0; i < 255; i++) {
        set_led(0, i, 0, 0);
        busy_wait_ms(1);
    }
	for (int i = 0; i < 255; i++) {
        set_led(0, 255-i, 0, 0);
        busy_wait_ms(1);
    }

    
    // Create Dumb Mode Task
    xTaskCreate(dumb_main, "DumbModeTask", 2048, NULL, 1, &xDumbModeHandle);
    // UBaseType_t uxCoreAffinityMask;
    // uxCoreAffinityMask = (( 1 << 0 ));
    // vTaskCoreAffinitySet(xDumbModeHandle, uxCoreAffinityMask);

    // nrf_init();
    //xTaskCreate(nrf_main, "NrfTask", 1024, NULL, 1, &xNrfHandle);
    // uxCoreAffinityMask = (( 1 << 1 ));
    // vTaskCoreAffinitySet(xNrfHandle, uxCoreAffinityMask);


    xTaskCreate(usb_main, "UsbTask", 2048, NULL, 1, &xUsbHandle);
    // uxCoreAffinityMask = (( 1 << 1 ));
    // vTaskCoreAffinitySet(xUsbHandle, uxCoreAffinityMask);

    // TEMP
    xTaskCreate(sensors_main, "SensorTask", 2048, NULL, 2, &xSensorHandle);
    // vTaskCoreAffinitySet(xSensorHandle, uxCoreAffinityMask);
    xTaskCreate(command_handler_main, "CommandTask", 2048, NULL, 2, &xCommandHandle);
    // vTaskCoreAffinitySet(xCommandHandle, uxCoreAffinityMask);

    // Start the scheduler
    vTaskStartScheduler();

    while (1) {

    }

    return 0;
}






















// // uROS
// #include <rcl/rcl.h>
// #include <rcl/error_handling.h>
// #include <rclc/rclc.h>
// #include <rclc/executor.h>
// #include <std_msgs/msg/int32.h>
// #include <rmw_microros/rmw_microros.h>
// extern "C" {
//     #include "pico_uart_transports.h"
// }

// // Local
// #include "sensors.h"
// #include "servos.h"
// #include "leg.hpp"

// // servo2040
// #include "servo2040.hpp"
// #include "analogmux.hpp"
// #include "analog.hpp"
// #include "button.hpp"

// using namespace plasma;
// using namespace servo;


// const float y_1 = 74;
// const float x_1 = round((y_1 / tan(M_PI / 6)) * 100) / 100;
// const float y_2 = 115;

// const uint16_t coxa_length = 65;
// const uint16_t femur_length = 120;
// const uint16_t tibia_length = 200;

// offset_t leg1_offset = {x_1, y_1, 0, 30};
// Leg leg1 = Leg(leg1_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_1, servo2040::SERVO_2, servo2040::SERVO_3);

// offset_t leg2_offset = {0, y_2, 90};
// Leg leg2 = Leg(leg2_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_4, servo2040::SERVO_5, servo2040::SERVO_6);

// offset_t leg3_offset = {-x_1, y_1, 120};
// Leg leg3 = Leg(leg3_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_7, servo2040::SERVO_8, servo2040::SERVO_9);

// offset_t leg4_offset = {-x_1, -y_1, 210};
// Leg leg4 = Leg(leg4_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_12, servo2040::SERVO_11, servo2040::SERVO_10);

// offset_t leg5_offset = {0, -y_2, 270};
// Leg leg5 = Leg(leg5_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_15, servo2040::SERVO_14, servo2040::SERVO_13);

// offset_t leg6_offset = {x_1, -y_1, 330};
// Leg leg6 = Leg(leg6_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_18, servo2040::SERVO_17, servo2040::SERVO_16);

// Leg *all_legs[6] = {&leg1, &leg2, &leg3, &leg4, &leg5, &leg6};

// int main() {

//     stdio_init_all();

//     // while (!tud_cdc_connected()) { sleep_ms(100);  }

//     leg1.init();
//     init_servos();
//     update_servos(all_legs);
//     sleep_ms(5000);

//     while (1) {
//         for (float i = 10; i < 50; i++) {
//             offset_t pos = {0, 0, 0, 0};
//             leg1.get_offset(&pos);

//             position_t position = {pos.X, pos.Y, pos.Z};
//             int16_t theta = 0 + pos.rotation;
//             position.X += ((leg1.get_coxa_length() + ((leg1.get_femur_length() + leg1.get_tibia_length()) / 2)) * cos(deg2rad(theta)) + (i));
//             position.Y += (leg1.get_coxa_length() + ((leg1.get_femur_length() + leg1.get_tibia_length()) / 2)) * sin(deg2rad(theta));
//             position.Z = 100;
//             leg1.set_leg_position(&position);
//             update_servos(all_legs);
//             sleep_ms(10);
//         }

//         for (float i = 50; i >= 10; i--) {
//             offset_t pos = {0, 0, 0, 0};
//             leg1.get_offset(&pos);

//             position_t position = {pos.X, pos.Y, pos.Z};
//             int16_t theta = 0 + pos.rotation;
//             position.X += (leg1.get_coxa_length() + ((leg1.get_femur_length() + leg1.get_tibia_length()) / 2) * cos(deg2rad(theta)) + (i));
//             position.Y += (leg1.get_coxa_length() + ((leg1.get_femur_length() + leg1.get_tibia_length()) / 2)) * sin(deg2rad(theta));
//             position.Z = 100;
//             leg1.set_leg_position(&position);
//             update_servos(all_legs);
//             sleep_ms(10);
//         }
//     }

//     // offset_t pos = {0, 0, 0, 0};
//     // leg1.get_offset(&pos);

//     // printf("%f - %f - %f - %f\n\r", pos.X, pos.Y, pos.Z, pos.rotation);

//     // position_t position = {pos.X, pos.Y, pos.Z};
//     // int16_t theta = 30 + pos.rotation;
//     // position.X += (leg1.get_coxa_length() + leg1.get_femur_length() + leg1.get_tibia_length()) * cos(deg2rad(theta));
//     // position.Y += (leg1.get_coxa_length() + leg1.get_femur_length() + leg1.get_tibia_length()) * sin(deg2rad(theta));
//     // position.Z = 0;
//     // leg1.set_leg_position(&position);

//     // leg1.get_leg_position(&position);
//     // printf("%f - %f - %f\n\r", position.X, position.Y, position.Z);

//     // for (uint8_t leg = 0; leg < 6; leg++) {
//     //     all_legs[leg].init();
//     // }
//     update_servos(all_legs);

//     sleep_ms(10000);

//     // vTaskStartScheduler();

//     while (1) {
        
//     }
// }