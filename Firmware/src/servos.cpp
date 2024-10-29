#include "servos.h"
#include <cstdio>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "errors.h"


const uint START_PIN = servo2040::SERVO_1;
const uint END_PIN = servo2040::SERVO_18;
const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;
ServoCluster servos = ServoCluster(pio0, 0, START_PIN, NUM_SERVOS);

static SemaphoreHandle_t servo_mutex;


uint8_t init_servos() {

    Calibration calibration = Calibration();
    calibration.apply_three_pairs(500, 1500, 2500, -(270/2), 0, (270/2));

    servos.create_servo_states(calibration, true);

    servos.init();

    // Create mutex
    servo_mutex = xSemaphoreCreateMutex();
    if (servo_mutex == NULL) {
        // Handle error: mutex creation failed
        // You might log an error or stop execution, depending on the application
    }

    sleep_ms(100);

    return 0;
}


uint8_t update_servos(Leg *legs[]) {

    for (uint8_t leg = 0; leg < 6; leg++) {
        if (legs[leg]->is_enabled()) {

            // printf("Coxa nr: %d - servo pos.: %f - ", legs[leg]->get_coxa_servo(), legs[leg]->get_coxa_position());
            // printf("Femur nr: %d - servo pos.: %f - ", legs[leg]->get_femur_servo(), legs[leg]->get_femur_position());
            // printf("Tibia nr: %d - servo pos.: %f\n\r", legs[leg]->get_tibia_servo(), legs[leg]->get_tibia_position());
            set_servo(legs[leg]->get_coxa_servo(), legs[leg]->get_coxa_position());
            set_servo(legs[leg]->get_femur_servo(), -legs[leg]->get_femur_position());
            set_servo(legs[leg]->get_tibia_servo(), legs[leg]->get_tibia_position());
        }
        else {
            servos.disable(legs[leg]->get_coxa_servo());
            servos.disable(legs[leg]->get_femur_servo());
            servos.disable(legs[leg]->get_tibia_servo());
        }
    }

    return 0;
}


uint8_t set_servo(uint8_t index, float value) {
    if (xSemaphoreTake(servo_mutex, portMAX_DELAY) == pdTRUE) {
        servos.value(index, value, true);
        xSemaphoreGive(servo_mutex);
    } 
    else {
        // Handle case where the mutex could not be taken
        // (e.g., timeout or an error, depending on your use case)
    }

    return ERR_NONE;
}