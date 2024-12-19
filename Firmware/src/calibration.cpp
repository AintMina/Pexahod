#include "calibration.h"
#include "robot.h"
#include "leg.hpp"
#include "servos.h"
#include "led.h"
#include "unit_conversion.h"

#include "FreeRTOS.h"
#include "task.h"


const float calibration_angles[3] = {0.0, -PI_OVER_2, 0.0};

void calibration_init() {

}

void calibration_main(void *pvParameters) {
    calibration_init();

    // Initialize the last wake time
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(1000); // 1000ms = 1 second

    while (1) {

        for (int i = 0; i < 6; i++) {
            all_legs[i]->set_coxa_position(calibration_angles[0]);
            all_legs[i]->set_femur_position(calibration_angles[1]);
            all_legs[i]->set_tibia_position(calibration_angles[2]);
        }

        update_servos(all_legs);

        for (int i = 0; i < 255; i++) {
            set_led(2, i, 0, 0);
            vTaskDelay(1);
        }
        for (int i = 0; i < 255; i++) {
            set_led(2, 255-i, 0, 0);
            vTaskDelay(1);
        }

        // Block the task until the next cycle
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
    
}