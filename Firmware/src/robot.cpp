#include "robot.hpp"


// Constructor
Robot::Robot() 
    : max_velocity(0), gait_cycle_time(0), gait_timestep(0), friction(0.0f) {
    this->velocity = {0.0f, 0.0f, 0.0f};
    this->rotation = {0.0f, 0.0f, 0.0f};
    this->position = {0, 0, 0};
    this->offset = {0, 0, 0};

    // Create mutex for thread safety
    this->mutex = xSemaphoreCreateMutex();
}

// Destructor
Robot::~Robot() {
    if (this->mutex != NULL) {
        vSemaphoreDelete(this->mutex);
    }
}

// Velocity control
void Robot::set_x_velocity(float x) {
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        this->velocity.X = x;
        xSemaphoreGive(this->mutex);
    }
}

void Robot::set_y_velocity(float y) {
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        this->velocity.Y = y;
        xSemaphoreGive(this->mutex);
    }
}

void Robot::set_rotation_velocity(float rotation) {
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        this->velocity.rotation = rotation;
        xSemaphoreGive(this->mutex);
    }
}

void Robot::set_velocity(float x, float y, float rotation) {
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        this->velocity.X = x;
        this->velocity.Y = y;
        this->velocity.rotation = rotation;
        xSemaphoreGive(this->mutex);
    }
}

velocity_t Robot::get_velocity() const {
    velocity_t vel;
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        vel = this->velocity;
        xSemaphoreGive(this->mutex);
    }
    return vel;
}

void Robot::set_max_velocity(uint16_t max_vel) {
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        this->max_velocity = max_vel;
        xSemaphoreGive(this->mutex);
    }
}

uint16_t Robot::get_max_velocity() const {
    uint16_t max_vel;
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        max_vel = this->max_velocity;
        xSemaphoreGive(this->mutex);
    }
    return max_vel;
}

// Rotation control
void Robot::set_rotation(float x, float y, float z) {
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        this->rotation.X = x;
        this->rotation.Y = y;
        this->rotation.Z = z;
        xSemaphoreGive(this->mutex);
    }
}

rotation_t Robot::get_rotation() const {
    rotation_t rot;
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        rot = this->rotation;
        xSemaphoreGive(this->mutex);
    }
    return rot;
}

// Position control
void Robot::set_position(int32_t x, int32_t y, int32_t z) {
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        this->position.X = x;
        this->position.Y = y;
        this->position.Z = z;
        xSemaphoreGive(this->mutex);
    }
}

position_t Robot::get_position() const {
    position_t pos;
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        pos = this->position;
        xSemaphoreGive(this->mutex);
    }
    return pos;
}

void Robot::set_offset(int32_t x, int32_t y, int32_t z) {
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        this->offset.X = x;
        this->offset.Y = y;
        this->offset.Z = z;
        xSemaphoreGive(this->mutex);
    }
}

position_t Robot::get_offset() const {
    position_t off;
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        off = this->offset;
        xSemaphoreGive(this->mutex);
    }
    return off;
}

// Gait control
void Robot::set_gait_cycle_time(int64_t cycle_time) {
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        this->gait_cycle_time = cycle_time;
        xSemaphoreGive(this->mutex);
    }
}

int64_t Robot::get_gait_cycle_time() const {
    int64_t cycle_time;
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        cycle_time = this->gait_cycle_time;
        xSemaphoreGive(this->mutex);
    }
    return cycle_time;
}

void Robot::set_gait_timestep(int64_t timestep) {
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        this->gait_timestep = timestep;
        xSemaphoreGive(this->mutex);
    }
}

int64_t Robot::get_gait_timestep() const {
    int64_t timestep;
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        timestep = this->gait_timestep;
        xSemaphoreGive(this->mutex);
    }
    return timestep;
}

// Friction control
void Robot::set_friction(float value) {
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        this->friction = value;
        xSemaphoreGive(this->mutex);
    }
}

float Robot::get_friction() const {
    float fric;
    if (xSemaphoreTake(this->mutex, portMAX_DELAY)) {
        fric = this->friction;
        xSemaphoreGive(this->mutex);
    }
    return fric;
}