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

#include "leg.hpp"
#include "errors.h"
#include "robot.h"

#include <math.h>
#include <string.h>


// Constructors
Leg::Leg(uint8_t id, offset_t offset, uint coxa, uint femur, uint tibia) {
    this->id = id;

    this->offset.X = offset.X;
    this->offset.Y = offset.Y;
    this->offset.Z = offset.Z;
    this->offset.rotation = offset.rotation;

    this->servos.coxa = coxa;
    this->servos.femur = femur;
    this->servos.tibia = tibia;

    this->gait_offset_point.X = this->offset.X + (this->gait_offset_length * cos(this->offset.rotation));
    this->gait_offset_point.Y = this->offset.Y + (this->gait_offset_length * sin(this->offset.rotation));
    this->gait_offset_point.Z = -150;

    if (this->gait_offset_point.X < 0.001 && this->gait_offset_point.X > -0.001) {
        this->gait_offset_point.X = 0.0;
    }
    if (this->gait_offset_point.Y < 0.001 && this->gait_offset_point.Y > -0.001) {
        this->gait_offset_point.Y = 0.0;
    }
    if (this->gait_offset_point.Z < 0.001 && this->gait_offset_point.Z > -0.001) {
        this->gait_offset_point.Z = 0.0;
    }
}

Leg::Leg(uint8_t id, offset_t offset, int16_t coxa_length, int16_t femur_length, int16_t tibia_length, uint coxa, uint femur, uint tibia) {
    this->id = id;
    
    this->offset.X = offset.X;
    this->offset.Y = offset.Y;
    this->offset.Z = offset.Z;
    this->offset.rotation = offset.rotation;

    this->coxa_length = coxa_length;
    this->femur_length = femur_length;
    this->tibia_length = tibia_length;

    this->servos.coxa = coxa;
    this->servos.femur = femur;
    this->servos.tibia = tibia;

    this->gait_offset_point.X = this->offset.X + (this->gait_offset_length * cos(this->offset.rotation));
    this->gait_offset_point.Y = this->offset.Y + (this->gait_offset_length * sin(this->offset.rotation));
    this->gait_offset_point.Z = -150;

    if (this->gait_offset_point.X < 0.001 && this->gait_offset_point.X > -0.001) {
        this->gait_offset_point.X = 0.0;
    }
    if (this->gait_offset_point.Y < 0.001 && this->gait_offset_point.Y > -0.001) {
        this->gait_offset_point.Y = 0.0;
    }
    if (this->gait_offset_point.Z < 0.001 && this->gait_offset_point.Z > -0.001) {
        this->gait_offset_point.Z = 0.0;
    }
}

Leg::~Leg() {
    if (mutex != NULL) {
        vSemaphoreDelete(mutex);
    }
}

uint8_t Leg::init() {
    // Create a mutex for this instance
    mutex = xSemaphoreCreateMutex();
    if (mutex == NULL) {
        // Handle error: failed to create mutex
    }

    this->enable();

    struct robot_settings_t settings;
    read_robot_settings(&settings);

    for (int i = 0; i < 3; i++) {
        this->servo_offsets[i] = settings.servo_offsets[this->id][i];
    }

    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->leg_position.X = this->coxa_length + this->femur_length + this->tibia_length;
        this->leg_position.Y = 0;
        this->leg_position.Z = 0;

        xSemaphoreGive(mutex); // Release the mutex
    }

    this->calculate_positions();

    return ERR_NONE;
}

uint8_t Leg::enable() {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->enabled = 1;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::disable() {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->enabled = 0;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::is_enabled() const {
    uint8_t ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->enabled;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_id(uint8_t new_id) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->id = new_id;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::get_id() const {
    uint8_t ret = 0xff;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->id;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_offset(offset_t offset) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->offset.X = offset.X;
        this->offset.Y = offset.Y;
        this->offset.rotation = offset.rotation;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::get_offset(offset_t *offset) const {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        offset->X = this->offset.X;
        offset->Y = this->offset.Y;
        offset->rotation = this->offset.rotation;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::set_coxa_length(uint16_t length) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->coxa_length = length;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint16_t Leg::get_coxa_length() const {
    uint16_t ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->coxa_length;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_femur_length(uint16_t length) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->femur_length = length;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint16_t Leg::get_femur_length() const {
    uint16_t ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->femur_length;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_tibia_length(uint16_t length) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->tibia_length = length;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint16_t Leg::get_tibia_length() const {
    uint16_t ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->tibia_length;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_gait(enum gaits_e gait) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->gait = gait;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

enum gaits_e Leg::get_gait() const {
    enum gaits_e ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->gait;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_max_step_size(uint16_t step_size) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->max_step_size = step_size;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint16_t Leg::get_max_step_size() const {
    uint16_t ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->max_step_size;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint16_t Leg::get_gait_offset_length() const {
    uint16_t ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->gait_offset_length;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_gait_offset(position_t offset) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->gait_offset_point.X = offset.X;
        this->gait_offset_point.Y = offset.Y;
        this->gait_offset_point.Z = offset.Z;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::get_gait_offset(position_t *offset) const {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        offset->X = this->gait_offset_point.X;
        offset->Y = this->gait_offset_point.Y;
        offset->Z = this->gait_offset_point.Z;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::set_coxa_position(float position) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->coxa_current_position = position;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

float Leg::get_coxa_position() const {
    float ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->coxa_current_position;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_femur_position(float position) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->femur_current_position = position;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

float Leg::get_femur_position() const {
    float ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->femur_current_position;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_tibia_position(float position) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->tibia_current_position = position;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

float Leg::get_tibia_position() const {
    float ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->tibia_current_position;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_coxa_limits(int16_t min, int16_t max) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->coxa_limits[0] = min;
        this->coxa_limits[1] = max;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::set_femur_limits(int16_t min, int16_t max) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->femur_limits[0] = min;
        this->femur_limits[1] = max;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::set_tibia_limits(int16_t min, int16_t max) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->tibia_limits[0] = min;
        this->tibia_limits[1] = max;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::set_leg_position(position_t *position) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        position_t new_position = {position->X, position->Y, -position->Z};
        new_position.X -= this->offset.X;
        new_position.Y -= this->offset.Y;

        float r = sqrt(pow(new_position.X, 2) + pow(new_position.Y, 2));
        float theta = atan2(new_position.Y, new_position.X);
        theta -= this->offset.rotation;
        
        new_position.X = cos(theta) * r;
        new_position.Y = sin(theta) * r;

        this->leg_position.X = new_position.X;
        this->leg_position.Y = new_position.Y;
        this->leg_position.Z = new_position.Z;

        xSemaphoreGive(mutex); // Release the mutex
    }

    this->calculate_positions();
    return ERR_NONE;
}

uint8_t Leg::set_leg_position_raw(position_t *position) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {

        this->leg_position.X = position->X;
        this->leg_position.Y = position->Y;
        this->leg_position.Z = -position->Z;

        xSemaphoreGive(mutex); // Release the mutex
    }

    this->calculate_positions();
    return ERR_NONE;
}

uint8_t Leg::get_leg_position(position_t *position) const {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        position->X = this->leg_position.X;
        position->Y = this->leg_position.Y;
        position->Z = -this->leg_position.Z;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::set_sensor_calculated_position(position_t position) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->sensor_calculated_position.X = position.X;
        this->sensor_calculated_position.Y = position.Y;
        this->sensor_calculated_position.Z = position.Z;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::get_sensor_calculated_position(position_t position) const {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        position.X = this->sensor_calculated_position.X;
        position.Y = this->sensor_calculated_position.Y;
        position.Z = this->sensor_calculated_position.Z;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::set_sensor_position(int16_t *position) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->sensor_position[0] = position[0];
        this->sensor_position[1] = position[1];
        this->sensor_position[2] = position[2];

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::get_sensor_position(int16_t *position) const {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        position[0] = this->sensor_position[0];
        position[1] = this->sensor_position[1];
        position[2] = this->sensor_position[2];

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::read_end_stop_sensor() {
    // TODO
    return ERR_NONE;
}

uint8_t Leg::get_end_stop_sensor() const {
    uint8_t ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->end_stop_sensor;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_coxa_servo(uint8_t servo) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->servos.coxa = servo;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::get_coxa_servo() const {
    uint8_t ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->servos.coxa;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_femur_servo(uint8_t servo) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->servos.femur = servo;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::get_femur_servo() const {
    uint8_t ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->servos.femur;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::set_tibia_servo(uint8_t servo) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        this->servos.tibia = servo;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}


uint8_t Leg::get_tibia_servo() const {
    uint8_t ret;
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        ret = this->servos.tibia;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ret;
}

uint8_t Leg::calculate_coxa_position() {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        float result = atan2(this->leg_position.Y, this->leg_position.X);

        this->coxa_current_position = result;

        xSemaphoreGive(mutex); // Release the mutex
    }
    return ERR_NONE;
}

uint8_t Leg::calculate_femur_position() {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        float new_x = sqrt(pow(this->leg_position.X, 2) + pow(this->leg_position.Y, 2));
        new_x -= this->coxa_length;

        float new_z = this->leg_position.Z;

        float first_part = (this->tibia_length * sin(this->tibia_current_position));
        float second_part = (this->femur_length + (this->tibia_length * cos(this->tibia_current_position)));

        if (second_part == 0) {
            second_part = 0.0001;
        }

        float B = atan2(first_part, second_part);
        float A = atan2(new_z, new_x);

        float result = A - B;

        this->femur_current_position = result;

        xSemaphoreGive(mutex); // Release the mutex
    }

    return ERR_NONE;
}

uint8_t Leg::calculate_tibia_position() {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        float new_x = sqrt(pow(this->leg_position.X, 2) + pow(this->leg_position.Y, 2));
        new_x -= this->coxa_length;

        float new_z = this->leg_position.Z;

        float first_part = (pow(new_x, 2) + pow(new_z, 2) - pow(this->femur_length, 2) - pow(this->tibia_length, 2));
        float second_part = (2 * this->tibia_length * this->femur_length);
        float division = first_part / second_part;

        if (division < -1.0) {
            division = -1.0;
        }
        else if (division > 1.0) {
            division = 1.0;
        }

        float result = acos(division);

        this->tibia_current_position = result;

        xSemaphoreGive(mutex); // Release the mutex
    }

    return ERR_NONE;
}

uint8_t Leg::calculate_positions() {
    this->calculate_coxa_position();
    this->calculate_tibia_position();
    this->calculate_femur_position();

    return ERR_NONE;
}

uint8_t Leg::set_servo_offsets(float offsets[3]) {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(this->servo_offsets, offsets, sizeof(float) * 3);

        xSemaphoreGive(mutex); // Release the mutex
    }

    return ERR_NONE;
}

uint8_t Leg::get_servo_offsets(float offsets[3]) const {
    if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(offsets, this->servo_offsets, sizeof(float) * 3);

        xSemaphoreGive(mutex); // Release the mutex
    }

    return ERR_NONE;
}