#ifndef LEG_HPP
#define LEG_HPP

#include <cstdint>
#include "servo2040.hpp"
using namespace servo;

struct offset_t {
    int32_t X;
    int32_t Y;
    int32_t rotation;
};

struct position_t {
    int32_t X;
    int32_t Y;
    int32_t Z;
};

class Leg {

private:
    uint8_t enabled = 0;
    offset_t offset = (0, 0, 0);  // (X,Y,rotation)
    uint16_t coxa_length = 0;
    uint16_t femur_length = 0;
    uint16_t tibia_length = 0;
    int16_t coxa_current_position = 0;
    int16_t femur_current_position = 0;
    int16_t tibia_current_position = 0;
    int16_t coxa_limits[2];  // (min,max)
    int16_t femur_limits[2];  // (min,max)
    int16_t tibia_limits[2];  // (min,max)
    position_t leg_position;  // (X,Y,Z)
    position_t calculated_position;  // (X,Y,Z)
    int16_t sensor_position[3] = (0, 0, 0);  // (coxa,femur,tibia)
    uint8_t end_stop_sensor = 0;
    Servo servos[3];  // (coxa,femur,tibia)
    uint8_t move = 0; // ???

public:
    // Constructors
    Leg() = default;
    Leg(offset_t offset, Servo coxa, Servo femur, Servo tibia);
    Leg(offset_t offset, int16_t coxa_length, int16_t femur_length, int16_t tibia_length, Servo coxa, Servo femur, Servo tibia);

    uint8_t init();

    uint8_t enable();
    uint8_t disable();
    uint8_t is_enabled() const;

    uint8_t set_offset(offset_t offset);
    uint8_t get_offset(offset_t offset) const;

    uint8_t set_coxa_length(uint16_t length);
    uint16_t get_coxa_length() const;
    uint8_t set_femur_length(uint16_t length);
    uint16_t get_femur_length() const;
    uint8_t set_tibia_length(uint16_t length);
    uint16_t get_tibia_length() const;

    uint8_t set_coxa_position(int16_t position);
    int16_t get_coxa_position() const;
    uint8_t set_femur_position(int16_t position);
    int16_t get_femur_position() const;
    uint8_t set_tibia_position(int16_t position);
    int16_t get_tibia_position() const;

    uint8_t set_coxa_limits(int16_t min, int16_t max);
    uint8_t set_femur_limits(int16_t min, int16_t max);
    uint8_t set_tibia_limits(int16_t min, int16_t max);

    uint8_t set_leg_position(position_t position);
    uint8_t get_leg_position(position_t position) const;
    uint8_t set_calculated_position(position_t position);
    uint8_t get_calculated_position(position_t position) const;
    
    uint8_t set_sensor_position(int16_t *position);
    uint8_t get_sensor_position(int16_t *position) const;
    uint8_t read_end_stop_sensor();
    uint8_t get_end_stop_sensor() const;

    uint8_t transform_xyz(position_t position);
    uint8_t calculate_coxa_position();
    uint8_t calculate_femur_position();
    uint8_t calculate_tibia_position();
    uint8_t calculate_positions();

    uint8_t move_coxa() const;
    uint8_t move_femur() const;
    uint8_t move_tibia() const;
    uint8_t move_leg() const;
};

#endif // LEG_HPP