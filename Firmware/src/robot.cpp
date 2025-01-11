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

// Local
#include "robot.h"
#include "unit_conversion.h"
#include "leg.hpp"

// System
#include <string.h>
#include <math.h>
#include "pico/stdlib.h"
#include "servo2040.hpp"


// Flash variables
const uint8_t *settings_pointer = (const uint8_t *) (XIP_BASE + FLASH_TARGET_OFFSET);

// Internal state variables
static velocity_t velocity = {0.0f, 0.0f, 0.0f};
static uint16_t max_velocity = 0;
static rotation_t rotation = {0.0f, 0.0f, 0.0f};
static position_t position = {0, 0, 0};
static position_t offset = {0, 0, 0};
static int64_t gait_cycle_time = 0;
static int64_t gait_timestep = 0;
static float friction = 0.0f;

// Mutex for thread safety
static SemaphoreHandle_t mutex = xSemaphoreCreateMutex();
SemaphoreHandle_t flash_mutex;
SemaphoreHandle_t flash_read_mutex;
SemaphoreHandle_t flash_write_mutex;

// Legs
using namespace plasma;
using namespace servo;

const float x_1 = 74;
const float y_1 = round((x_1 / tan(2*PI / 3)) * 100) / 100;
const float x_2 = 115;

const uint16_t coxa_length = 65;
const uint16_t femur_length = 120;
const uint16_t tibia_length = 200;

offset_t leg1_offset = {-x_1, y_1, 0, (float)(deg_to_rad(120.0))};
Leg leg1 = Leg(0, leg1_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_1, servo2040::SERVO_2, servo2040::SERVO_3);

offset_t leg2_offset = {-x_2, 0, 0, (float)(deg_to_rad(180.0))};
Leg leg2 = Leg(1, leg2_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_4, servo2040::SERVO_5, servo2040::SERVO_6);

offset_t leg3_offset = {-x_1, -y_1, 0, (float)(deg_to_rad(240.0))};
Leg leg3 = Leg(2, leg3_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_7, servo2040::SERVO_8, servo2040::SERVO_9);

offset_t leg4_offset = {x_1, -y_1, 0, (float)(deg_to_rad(300.0))};
Leg leg4 = Leg(3, leg4_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_10, servo2040::SERVO_11, servo2040::SERVO_12);

offset_t leg5_offset = {x_2, 0, 0, (float)(deg_to_rad(0.0))};
Leg leg5 = Leg(4, leg5_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_13, servo2040::SERVO_14, servo2040::SERVO_15);

offset_t leg6_offset = {x_1, y_1, 0, (float)(deg_to_rad(60.0))};
Leg leg6 = Leg(5, leg6_offset, coxa_length, femur_length, tibia_length, servo2040::SERVO_16, servo2040::SERVO_17, servo2040::SERVO_18);

Leg *all_legs[6] = {&leg1, &leg2, &leg3, &leg4, &leg5, &leg6};


void init_robot_settings() {
    // Initialize the mutex
    flash_mutex = xSemaphoreCreateMutex();
    flash_read_mutex = xSemaphoreCreateMutex();
    flash_write_mutex = xSemaphoreCreateMutex();
    if (flash_mutex == NULL || flash_read_mutex == NULL || flash_write_mutex == NULL) {
        // Handle error: mutex creation failed
        // You might log an error or stop execution, depending on the application
    }
}

void read_robot_settings(struct robot_settings_t *buffer) {
    if (xSemaphoreTake(flash_read_mutex, portMAX_DELAY) == pdTRUE) {
        memcpy(buffer, settings_pointer, sizeof(struct robot_settings_t));

        if (buffer->version == 0 || buffer->version == 0xFFFFFFFF || buffer->version < SETTINGS_VERSION) {
            set_default_robot_settings();
            memcpy(buffer, settings_pointer, sizeof(struct robot_settings_t));
        }

        // Release the mutex
        xSemaphoreGive(flash_read_mutex);
    }
    else {
        // Handle case where the mutex could not be taken
        // (e.g., timeout or an error, depending on your use case)
    }
}

void __not_in_flash_func(write_robot_settings)(struct robot_settings_t *buffer) {
    if (xSemaphoreTake(flash_write_mutex, portMAX_DELAY) == pdTRUE) {
        // Critical section: Disable interrupts for SMP safety
        uint32_t ints = save_and_disable_interrupts();

        // Write 0xFF to the rest of it to hopefully perserve flash ??
        uint8_t write_buffer[BLOCK_SIZE];
        memset(write_buffer, 0xFF, BLOCK_SIZE);
        memcpy(write_buffer, buffer, sizeof(struct robot_settings_t));

        flash_range_erase(FLASH_TARGET_OFFSET, BLOCK_SIZE);
        flash_range_program(FLASH_TARGET_OFFSET, (const uint8_t *)write_buffer, BLOCK_SIZE);

        // Release interrupts and mutexes
        restore_interrupts(ints);
        xSemaphoreGive(flash_write_mutex);
    }
    else {
        // Handle case where the mutex could not be taken
        // (e.g., timeout or an error, depending on your use case)
    }
}

void set_default_robot_settings() {
    struct robot_settings_t buffer;
    memset(&buffer, 0, sizeof(struct robot_settings_t));
    buffer.version = SETTINGS_VERSION;
    buffer.default_mode = READY_MODE;

    write_robot_settings(&buffer);
}

void save_robot_settings() {
    struct robot_settings_t buffer;
    memset(&buffer, 0, sizeof(struct robot_settings_t));
    buffer.version = SETTINGS_VERSION;
    buffer.default_mode = READY_MODE;

    for (int i = 0; i < 6; i++) {
        all_legs[i]->get_servo_offsets(buffer.servo_offsets[i * 3]);
    }

    write_robot_settings(&buffer);
}

// Velocity control functions
void set_x_velocity(float x) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        velocity.X = x;
        xSemaphoreGive(mutex);
    }
}

void set_y_velocity(float y) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        velocity.Y = y;
        xSemaphoreGive(mutex);
    }
}

void set_rotation_velocity(float rotation_val) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        velocity.rotation = rotation_val;
        xSemaphoreGive(mutex);
    }
}

void set_velocity(float x, float y, float rotation_val) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        velocity.X = x;
        velocity.Y = y;
        velocity.rotation = rotation_val;
        xSemaphoreGive(mutex);
    }
}

velocity_t get_velocity() {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        velocity_t vel = velocity;
        xSemaphoreGive(mutex);
        return vel;
    } else {
        // Handle timeout scenario, e.g., return a default value or error state
        return {0.0f, 0.0f, 0.0f}; // Default or error value
    }
}

void set_max_velocity(uint16_t max_vel) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        max_velocity = max_vel;
        xSemaphoreGive(mutex);
    }
}

uint16_t get_max_velocity() {
    uint16_t ret = 0;
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        ret = max_velocity;
        xSemaphoreGive(mutex);
    }
    return ret;
}

// Rotation control functions
void set_rotation(float x, float y, float z) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        rotation.X = x;
        rotation.Y = y;
        rotation.Z = z;
        xSemaphoreGive(mutex);
    }
}

rotation_t get_rotation() {
    rotation_t ret = {0, 0, 0};
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        ret = rotation;
        xSemaphoreGive(mutex);
    }
    return ret;
}

// Position control functions
void set_position(int32_t x, int32_t y, int32_t z) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        position.X = x;
        position.Y = y;
        position.Z = z;
        xSemaphoreGive(mutex);
    }
}

void get_position(position_t *pos) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        pos->X = position.X;
        pos->Y = position.Y;
        pos->Z = position.Z;
        xSemaphoreGive(mutex);
    }
}

void set_offset(int32_t x, int32_t y, int32_t z) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        offset.X = x;
        offset.Y = y;
        offset.Z = z;
        xSemaphoreGive(mutex);
    }
}

void get_offset(position_t *offs) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        offs->X = offset.X;
        offs->Y = offset.Y;
        offs->Z = offset.Z;
        xSemaphoreGive(mutex);
    }
}

// Gait control functions
void set_gait_cycle_time(int64_t cycle_time) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        gait_cycle_time = cycle_time;
        xSemaphoreGive(mutex);
    }
}

int64_t get_gait_cycle_time() {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        int64_t cycle_time = gait_cycle_time;
        xSemaphoreGive(mutex);
        return cycle_time;
    }
    else {
        return 0;
    }
}

void set_gait_timestep(int64_t timestep) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        gait_timestep = timestep;
        xSemaphoreGive(mutex);
    }
}

int64_t get_gait_timestep() {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        int64_t timestep = gait_timestep;
        xSemaphoreGive(mutex);
        return timestep;
    }
    else {
        return 0;
    }
}

// Friction control functions
void set_friction(float value) {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        friction = value;
        xSemaphoreGive(mutex);
    }
}

float get_friction() {
    if (xSemaphoreTake(mutex, pdMS_TO_TICKS(100)) == pdTRUE) { // 100 ms timeout
        float fric = friction;
        xSemaphoreGive(mutex);
        return fric;
    }
    else {
        return 0.0;
    }
}