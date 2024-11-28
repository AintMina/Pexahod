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

#include "robot_controller.h"
#include "mode_handler.h"
#include "led.h"
#include "gait.hpp"
#include "servos.h"
#include "leg.hpp"
#include "servo2040.hpp"
#include "robot.h"
#include "unit_conversion.h"

#include "FreeRTOS.h"
#include "task.h"
#include "pico/stdlib.h"

using namespace plasma;
using namespace servo;


#define PI 3.14159

// const float y_1 = 74;
// const float x_1 = round((y_1 / tan(PI / 6)) * 100) / 100;
// const float y_2 = 115;

const float x_1 = 74;
const float y_1 = round((x_1 / tan(2*PI / 3)) * 100) / 100;
const float x_2 = 115;

const uint16_t coxa_length = 65;
const uint16_t femur_length = 120;
const uint16_t tibia_length = 200;

offset_t leg1_offset = {-x_1, y_1, 0, (float)(deg_to_rad(120.0))};
Leg leg1 = Leg(leg1_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_1, servo2040::SERVO_2, servo2040::SERVO_3);

offset_t leg2_offset = {-x_2, 0, 0, (float)(deg_to_rad(180.0))};
Leg leg2 = Leg(leg2_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_4, servo2040::SERVO_5, servo2040::SERVO_6);

offset_t leg3_offset = {-x_1, -y_1, 0, (float)(deg_to_rad(240.0))};
Leg leg3 = Leg(leg3_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_7, servo2040::SERVO_8, servo2040::SERVO_9);

offset_t leg4_offset = {x_1, -y_1, 0, (float)(deg_to_rad(300.0))};
Leg leg4 = Leg(leg4_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_12, servo2040::SERVO_11, servo2040::SERVO_10);

offset_t leg5_offset = {x_2, 0, 0, (float)(deg_to_rad(0.0))};
Leg leg5 = Leg(leg5_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_15, servo2040::SERVO_14, servo2040::SERVO_13);

offset_t leg6_offset = {x_1, y_1, 0, (float)(deg_to_rad(60.0))};
Leg leg6 = Leg(leg6_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_18, servo2040::SERVO_17, servo2040::SERVO_16);

Leg *all_legs[6] = {&leg1, &leg2, &leg3, &leg4, &leg5, &leg6};


const float max_time_step = 1.0;
float time_step = 0.0;

void robot_controller_main(void *pvParameters) {

    leg1.init();
    leg2.init();
    leg3.init();
    leg4.init();
    leg5.init();
    leg6.init();
    init_servos();
    update_servos(all_legs);
    position_t positions[6];
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    // Initialize the last wake time
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10); // 1000ms = 1 second

    TickType_t xLastUpdateTime = xTaskGetTickCount();

    // Infinite loop for task
    while (1) {
        velocity_t vel = get_velocity();
        float vel_scalar = sqrt(pow(vel.X, 2) + pow(vel.Y, 2));
        float vel_angle = atan2(vel.Y, vel.X);
        set_led(5, 255*time_step, 0, 0);

        get_gait_point_v2(positions, all_legs, time_step, vel);
        leg1.set_leg_position(&positions[0]);
        leg2.set_leg_position(&positions[1]);
        leg3.set_leg_position(&positions[2]);
        leg4.set_leg_position(&positions[3]);
        leg5.set_leg_position(&positions[4]);
        leg6.set_leg_position(&positions[5]);
        update_servos(all_legs);

        TickType_t xCurrentTick = xTaskGetTickCount();
        TickType_t xDeltaTimeTicks = xCurrentTick - xLastUpdateTime;
        float fDeltaTimeSeconds = xDeltaTimeTicks * portTICK_PERIOD_MS / 1000.0f;
        xLastUpdateTime = xCurrentTick;

        if (fDeltaTimeSeconds > 0.1) {
            fDeltaTimeSeconds = 0.0;
        }

        time_step += vel_scalar * max_time_step * fDeltaTimeSeconds;
        if (time_step >= 1.0) {
            time_step = 0.0;
        }
        else if (time_step < 0.0) {
            time_step = 1.0;
        }

        // Block the task until the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }

}