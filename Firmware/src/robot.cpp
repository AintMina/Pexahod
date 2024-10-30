#include "robot.hpp"
#include <FreeRTOS.h>
#include <semphr.h>

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

// Velocity control functions
void set_x_velocity(float x) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    velocity.X = x;
    xSemaphoreGive(mutex);
}

void set_y_velocity(float y) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    velocity.Y = y;
    xSemaphoreGive(mutex);
}

void set_rotation_velocity(float rotation_val) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    velocity.rotation = rotation_val;
    xSemaphoreGive(mutex);
}

void set_velocity(float x, float y, float rotation_val) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    velocity.X = x;
    velocity.Y = y;
    velocity.rotation = rotation_val;
    xSemaphoreGive(mutex);
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
    xSemaphoreTake(mutex, portMAX_DELAY);
    max_velocity = max_vel;
    xSemaphoreGive(mutex);
}

uint16_t get_max_velocity() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    uint16_t max_vel = max_velocity;
    xSemaphoreGive(mutex);
    return max_vel;
}

// Rotation control functions
void set_rotation(float x, float y, float z) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    rotation.X = x;
    rotation.Y = y;
    rotation.Z = z;
    xSemaphoreGive(mutex);
}

rotation_t get_rotation() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    rotation_t rot = rotation;
    xSemaphoreGive(mutex);
    return rot;
}

// Position control functions
void set_position(int32_t x, int32_t y, int32_t z) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    position.X = x;
    position.Y = y;
    position.Z = z;
    xSemaphoreGive(mutex);
}

position_t get_position() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    position_t pos = position;
    xSemaphoreGive(mutex);
    return pos;
}

void set_offset(int32_t x, int32_t y, int32_t z) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    offset.X = x;
    offset.Y = y;
    offset.Z = z;
    xSemaphoreGive(mutex);
}

position_t get_offset() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    position_t offs = offset;
    xSemaphoreGive(mutex);
    return offs;
}

// Gait control functions
void set_gait_cycle_time(int64_t cycle_time) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    gait_cycle_time = cycle_time;
    xSemaphoreGive(mutex);
}

int64_t get_gait_cycle_time() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    int64_t cycle_time = gait_cycle_time;
    xSemaphoreGive(mutex);
    return cycle_time;
}

void set_gait_timestep(int64_t timestep) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    gait_timestep = timestep;
    xSemaphoreGive(mutex);
}

int64_t get_gait_timestep() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    int64_t timestep = gait_timestep;
    xSemaphoreGive(mutex);
    return timestep;
}

// Friction control functions
void set_friction(float value) {
    xSemaphoreTake(mutex, portMAX_DELAY);
    friction = value;
    xSemaphoreGive(mutex);
}

float get_friction() {
    xSemaphoreTake(mutex, portMAX_DELAY);
    float fric = friction;
    xSemaphoreGive(mutex);
    return fric;
}