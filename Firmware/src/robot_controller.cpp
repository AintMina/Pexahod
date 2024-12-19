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


const float max_time_step = 1.0;
float time_step = 0.0;

void robot_controller_main(void *pvParameters) {

    leg1.init();
    leg2.init();
    leg3.init();
    leg4.init();
    leg4.disable();
    leg5.init();
    leg6.init();
    init_servos();
    update_servos(all_legs);
    position_t positions[6];
    vTaskDelay(10000 / portTICK_PERIOD_MS);

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