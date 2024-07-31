#include "leg.hpp"
#include "errors.h"
#include <math.h>


double deg2rad(double degrees) {
    return degrees * (M_PI / 180.0);
}

double rad2deg(double radians) {
    return radians * (180.0 / M_PI);
}

// Constructors
Leg::Leg(offset_t offset, uint coxa, uint femur, uint tibia) {
    this->offset.X = offset.X;
    this->offset.Y = offset.Y;
    this->offset.Z = offset.Z;
    this->offset.rotation = offset.rotation;

    this->servos.coxa = coxa;
    this->servos.femur = femur;
    this->servos.tibia = tibia;
}
Leg::Leg(offset_t offset, int16_t coxa_length, int16_t femur_length, int16_t tibia_length, uint coxa, uint femur, uint tibia) {
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
}

uint8_t Leg::init() {
    this->enable();

    this->leg_position.X = this->coxa_length + this->femur_length + this->tibia_length;
    this->leg_position.Y = 0;
    this->leg_position.Z = 0;

    this->calculate_positions();

    return ERR_NONE;
}

uint8_t Leg::enable() {
    this->enabled = 1;
    return ERR_NONE;
}

uint8_t Leg::disable() {
    this->enabled = 0;
    return ERR_NONE;
}

uint8_t Leg::is_enabled() const {
    return this->enabled;
}

uint8_t Leg::set_offset(offset_t offset) {
    this->offset.X = offset.X;
    this->offset.Y = offset.Y;
    this->offset.rotation = offset.rotation;
    return ERR_NONE;
}

uint8_t Leg::get_offset(offset_t *offset) const {
    offset->X = this->offset.X;
    offset->Y = this->offset.Y;
    offset->rotation = this->offset.rotation;
    return ERR_NONE;
}

uint8_t Leg::set_coxa_length(uint16_t length) {
    this->coxa_length = length;
    return ERR_NONE;
}

uint16_t Leg::get_coxa_length() const {
    return this->coxa_length;
}

uint8_t Leg::set_femur_length(uint16_t length) {
    this->femur_length = length;
    return ERR_NONE;
}

uint16_t Leg::get_femur_length() const {
    return this->femur_length;
}

uint8_t Leg::set_tibia_length(uint16_t length) {
    this->tibia_length = length;
    return ERR_NONE;
}

uint16_t Leg::get_tibia_length() const {
    return this->tibia_length;
}

uint8_t Leg::set_coxa_position(float position) {
    this->coxa_current_position = position;
    return ERR_NONE;
}

float Leg::get_coxa_position() const {
    return this->coxa_current_position;
}

uint8_t Leg::set_femur_position(float position) {
    this->femur_current_position = position;
    return ERR_NONE;
}

float Leg::get_femur_position() const {
    return this->femur_current_position;
}

uint8_t Leg::set_tibia_position(float position) {
    this->tibia_current_position = position;
    return ERR_NONE;
}

float Leg::get_tibia_position() const {
    return this->tibia_current_position;
}

uint8_t Leg::set_coxa_limits(int16_t min, int16_t max) {
    this->coxa_limits[0] = min;
    this->coxa_limits[1] = max;
    return ERR_NONE;
}

uint8_t Leg::set_femur_limits(int16_t min, int16_t max) {
    this->femur_limits[0] = min;
    this->femur_limits[1] = max;
    return ERR_NONE;
}

uint8_t Leg::set_tibia_limits(int16_t min, int16_t max) {
    this->tibia_limits[0] = min;
    this->tibia_limits[1] = max;
    return ERR_NONE;
}

uint8_t Leg::set_leg_position(position_t *position) {
    position_t new_position = {position->X, position->Y, position->Z};
    new_position.X -= this->offset.X;
    new_position.Y -= this->offset.Y;

    float r = sqrt(pow(new_position.X, 2) + pow(new_position.Y, 2));
    float theta = atan2(new_position.Y, new_position.X);
    theta -= deg2rad(this->offset.rotation);
    
    new_position.X = cos(theta) * r;
    new_position.Y = sin(theta) * r;

    this->leg_position.X = new_position.X;
    this->leg_position.Y = new_position.Y;
    this->leg_position.Z = new_position.Z;

    this->calculate_positions();
    return ERR_NONE;
}

uint8_t Leg::get_leg_position(position_t *position) const {
    position->X = this->leg_position.X;
    position->Y = this->leg_position.Y;
    position->Z = this->leg_position.Z;
    return ERR_NONE;
}

uint8_t Leg::set_sensor_calculated_position(position_t position) {
    this->sensor_calculated_position.X = position.X;
    this->sensor_calculated_position.Y = position.Y;
    this->sensor_calculated_position.Z = position.Z;
    return ERR_NONE;
}

uint8_t Leg::get_sensor_calculated_position(position_t position) const {
    position.X = this->sensor_calculated_position.X;
    position.Y = this->sensor_calculated_position.Y;
    position.Z = this->sensor_calculated_position.Z;
    return ERR_NONE;
}

uint8_t Leg::set_sensor_position(int16_t *position) {
    this->sensor_position[0] = position[0];
    this->sensor_position[1] = position[1];
    this->sensor_position[2] = position[2];
    return ERR_NONE;
}

uint8_t Leg::get_sensor_position(int16_t *position) const {
    position[0] = this->sensor_position[0];
    position[1] = this->sensor_position[1];
    position[2] = this->sensor_position[2];
    return ERR_NONE;
}

uint8_t Leg::read_end_stop_sensor() {
    return ERR_NONE;
}

uint8_t Leg::get_end_stop_sensor() const {
    return this->end_stop_sensor;
}

uint8_t Leg::set_coxa_servo(uint8_t servo) {
    this->servos.coxa = servo;
    return ERR_NONE;
}

uint8_t Leg::get_coxa_servo() const {
    return this->servos.coxa;
}

uint8_t Leg::set_femur_servo(uint8_t servo) {
    this->servos.femur = servo;
    return ERR_NONE;
}

uint8_t Leg::get_femur_servo() const {
    return this->servos.femur;
}

uint8_t Leg::set_tibia_servo(uint8_t servo) {
    this->servos.tibia = servo;
    return ERR_NONE;
}


uint8_t Leg::get_tibia_servo() const {
    return this->servos.tibia;
}

uint8_t Leg::calculate_coxa_position() {
    float result = atan2(this->leg_position.Y, this->leg_position.X);

    if (result > (M_PI)) {
        result -= 2 * M_PI;
    }

    this->coxa_current_position = rad2deg(result);
    return ERR_NONE;
}

uint8_t Leg::calculate_femur_position() {
    float new_x = sqrt(pow(this->leg_position.X, 2) + pow(this->leg_position.Y, 2));
    new_x -= this->coxa_length;

    float new_z = this->leg_position.Z;

    float first_part = (this->tibia_length * sin(deg2rad(this->tibia_current_position)));
    float second_part = (this->femur_length + (this->tibia_length * cos(deg2rad(this->tibia_current_position))));
    float division = first_part / second_part;
    float B = atan(division);

    float A = atan2(new_z, new_x);

    if (A > M_PI) {
        A -= 2 * M_PI;
    }

    float result = A - B;

    this->femur_current_position = rad2deg(result);

    return ERR_NONE;
}

uint8_t Leg::calculate_tibia_position() {
    float new_x = sqrt(pow(this->leg_position.X, 2) + pow(this->leg_position.Y, 2));
    new_x -= this->coxa_length;

    float new_z = this->leg_position.Z;

    float first_part = (pow(new_x, 2) + pow(new_z, 2) - pow(this->femur_length, 2) - pow(this->tibia_length, 2));
    float second_part = (2 * this->tibia_length * this->femur_length);
    float division = first_part / second_part;
    float result = acos(division);

    this->tibia_current_position = rad2deg(result);

    return ERR_NONE;
}

uint8_t Leg::calculate_positions() {
    this->calculate_coxa_position();
    this->calculate_tibia_position();
    this->calculate_femur_position();

    return ERR_NONE;
}