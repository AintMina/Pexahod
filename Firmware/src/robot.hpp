#ifndef ROBOT_HPP
#define ROBOT_HPP

#include <stdint.h>

struct velocity_t {
    float X;
    float Y;
    float rotation;
};

struct rotation_t {
    float X;
    float Y;
    float Z;
};

struct position_t {
    int32_t X;
    int32_t Y;
    int32_t Z;
};

// Velocity control
void set_x_velocity(float x);
void set_y_velocity(float y);
void set_rotation_velocity(float rotation);
void set_velocity(float x, float y, float rotation);
velocity_t get_velocity();
void set_max_velocity(uint16_t max_vel);
uint16_t get_max_velocity();

// Rotation control
void set_rotation(float x, float y, float z);
rotation_t get_rotation();

// Position control
void set_position(int32_t x, int32_t y, int32_t z);
position_t get_position();
void set_offset(int32_t x, int32_t y, int32_t z);
position_t get_offset();

// Gait control
void set_gait_cycle_time(int64_t cycle_time);
int64_t get_gait_cycle_time();
void set_gait_timestep(int64_t timestep);
int64_t get_gait_timestep();

// Friction control
void set_friction(float value);
float get_friction();

#endif // ROBOT_HPP