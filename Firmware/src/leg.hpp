#ifndef LEG_HPP
#define LEG_HPP

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

#include <cstdint>
#include "pico/stdlib.h"
#include "robot.hpp"
#include "gait.hpp"

struct offset_t {
    float X;
    float Y;
    float Z;
    float rotation;
};

// From SERVO_1 to SERVO_18
struct servos_t {
    uint8_t coxa;
    uint8_t femur;
    uint8_t tibia;
};


double deg2rad(double degrees);
double rad2deg(double radians);

class Leg {

private:
    uint8_t enabled = 0;
    offset_t offset = {0, 0, 0, 0};  // (X,Y,rotation)
    uint16_t coxa_length = 65;
    uint16_t femur_length = 120;
    uint16_t tibia_length = 200;
    float coxa_current_position = 0;
    float femur_current_position = 0;
    float tibia_current_position = 0;
    int16_t coxa_limits[2];  // (min,max)
    int16_t femur_limits[2];  // (min,max)
    int16_t tibia_limits[2];  // (min,max)
    position_t leg_position;  // (X,Y,Z)
    position_t sensor_calculated_position;  // (X,Y,Z)
    int16_t sensor_position[3] = {0, 0, 0};  // (coxa,femur,tibia)
    uint8_t end_stop_sensor = 0;
    servos_t servos;  // (coxa,femur,tibia)
    uint8_t move = 0; // ???
    uint16_t gait_offset_length = coxa_length + ((femur_length + tibia_length) / 2);
    position_t gait_offset_point;
    uint16_t max_step_size = 100;
    enum Gaits gait = TRIGAIT;

public:
    // Constructors
    Leg() = default;
    Leg(offset_t offset, uint coxa, uint femur, uint tibia);
    Leg(offset_t offset, int16_t coxa_length, int16_t femur_length, int16_t tibia_length, uint coxa, uint femur, uint tibia);

    uint8_t init();

    uint8_t enable();
    uint8_t disable();
    uint8_t is_enabled() const;

    uint8_t set_offset(offset_t offset);
    uint8_t get_offset(offset_t *offset) const;

    uint8_t set_coxa_length(uint16_t length);
    uint16_t get_coxa_length() const;
    uint8_t set_femur_length(uint16_t length);
    uint16_t get_femur_length() const;
    uint8_t set_tibia_length(uint16_t length);
    uint16_t get_tibia_length() const;

    uint8_t set_gait(enum Gaits gait);
    enum Gaits get_gait() const;
    uint8_t set_max_step_size(uint16_t step_size);
    uint16_t get_max_step_size() const;
    uint8_t set_gait_offset(position_t offset);
    uint8_t get_gait_offset(position_t *offset) const;

    uint8_t set_coxa_position(float position);
    float get_coxa_position() const;
    uint8_t set_femur_position(float position);
    float get_femur_position() const;
    uint8_t set_tibia_position(float position);
    float get_tibia_position() const;

    uint8_t set_coxa_limits(int16_t min, int16_t max);
    uint8_t set_femur_limits(int16_t min, int16_t max);
    uint8_t set_tibia_limits(int16_t min, int16_t max);

    uint8_t set_leg_position(position_t *position);
    uint8_t get_leg_position(position_t *position) const;
    uint8_t set_sensor_calculated_position(position_t position);
    uint8_t get_sensor_calculated_position(position_t position) const;
    
    uint8_t set_sensor_position(int16_t *position);
    uint8_t get_sensor_position(int16_t *position) const;
    uint8_t read_end_stop_sensor();
    uint8_t get_end_stop_sensor() const;

    uint8_t set_coxa_servo(uint8_t servo);
    uint8_t get_coxa_servo() const;
    uint8_t set_femur_servo(uint8_t servo);
    uint8_t get_femur_servo() const;
    uint8_t set_tibia_servo(uint8_t servo);
    uint8_t get_tibia_servo() const;

    uint8_t calculate_coxa_position();
    uint8_t calculate_femur_position();
    uint8_t calculate_tibia_position();
    uint8_t calculate_positions();
};

#endif // LEG_HPP