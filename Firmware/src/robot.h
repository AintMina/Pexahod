#ifndef ROBOT_H
#define ROBOT_H



extern "C" {
  #include <hardware/sync.h>
  #include <hardware/flash.h>
};

#include "mode_handler.h"
#include <stdint.h>

// FreeRTOS
#include <FreeRTOS.h>
#include <semphr.h>


#define FLASH_TARGET_OFFSET (256 * 1024)
// #define FLASH_TARGET_OFFSET (PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE)
#define BLOCK_SIZE FLASH_SECTOR_SIZE         // TODO: change to match the flash
#define SETTINGS_VERSION 1U


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

struct robot_settings_t {
    uint32_t version;
    enum mode_e default_mode;
    float servo_offsets[6][3]; // Joint offsets (leg1,leg2,leg3,leg4,leg5,leg6)(coxa,femur,tibia)
};

class Leg;
extern Leg leg1;
extern Leg leg2;
extern Leg leg3;
extern Leg leg4;
extern Leg leg5;
extern Leg leg6;
extern Leg *all_legs[6];
extern SemaphoreHandle_t leg_mutexes[6];


void init_robot_settings();
void read_robot_settings(struct robot_settings_t *buffer);
void write_robot_settings(struct robot_settings_t *buffer);
void set_default_robot_settings();
void save_robot_settings();

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
void get_position(position_t *pos);
void set_offset(int32_t x, int32_t y, int32_t z);
void get_offset(position_t *offs);

// Gait control
void set_gait_cycle_time(int64_t cycle_time);
int64_t get_gait_cycle_time();
void set_gait_timestep(int64_t timestep);
int64_t get_gait_timestep();

// Friction control
void set_friction(float value);
float get_friction();



#endif // ROBOT_H