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

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// uROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>
extern "C" {
    #include "pico_uart_transports.h"
}

// Local
#include "sensors.h"
#include "servos.h"
#include "leg.hpp"

// servo2040
#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"
#include "button.hpp"

using namespace plasma;
using namespace servo;


const float x_1 = 74;
const float y_1 = round((x_1 / tan(M_PI / 6)) * 100) / 100;
const float x_2 = 115;

const uint16_t coxa_length = 65;
const uint16_t femur_length = 120;
const uint16_t tibia_length = 200;

offset_t leg1_offset = {-x_1, y_1, 0, 120};
Leg leg1 = Leg(leg1_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_1, servo2040::SERVO_2, servo2040::SERVO_3);

offset_t leg2_offset = {-x_2, 0, 180};
Leg leg2 = Leg(leg2_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_4, servo2040::SERVO_5, servo2040::SERVO_6);

offset_t leg3_offset = {-x_1, -y_1, 240};
Leg leg3 = Leg(leg3_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_7, servo2040::SERVO_8, servo2040::SERVO_9);

offset_t leg4_offset = {x_1, -y_1, 300};
Leg leg4 = Leg(leg4_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_12, servo2040::SERVO_11, servo2040::SERVO_10);

offset_t leg5_offset = {x_2, 0, 0};
Leg leg5 = Leg(leg5_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_15, servo2040::SERVO_14, servo2040::SERVO_13);

offset_t leg6_offset = {x_1, y_1, 60};
Leg leg6 = Leg(leg6_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_18, servo2040::SERVO_17, servo2040::SERVO_16);

Leg all_legs[6] = {leg1, leg2, leg3, leg4, leg5, leg6};

int main() {
    stdio_init_all();
}