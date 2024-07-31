#include "leg.hpp"
#include "errors.h"
#include <math.h>

// Constructors
Leg::Leg(offset_t offset, uint coxa, uint femur, uint tibia) {}
Leg::Leg(offset_t offset, int16_t coxa_length, int16_t femur_length, int16_t tibia_length, uint coxa, uint femur, uint tibia) {}

uint8_t Leg::init() {
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

uint8_t Leg::get_offset(offset_t offset) const {
    offset.X = this->offset.X;
    offset.Y = this->offset.Y;
    offset.rotation = this->offset.rotation;
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

uint8_t Leg::set_coxa_position(int16_t position) {
    this->coxa_current_position = position;
    return ERR_NONE;
}

int16_t Leg::get_coxa_position() const {
    return this->coxa_current_position;
}

uint8_t Leg::set_femur_position(int16_t position) {
    this->femur_current_position = position;
    return ERR_NONE;
}

int16_t Leg::get_femur_position() const {
    return this->femur_current_position;
}

uint8_t Leg::set_tibia_position(int16_t position) {
    this->tibia_current_position = position;
    return ERR_NONE;
}

int16_t Leg::get_tibia_position() const {
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

uint8_t Leg::set_leg_position(position_t position) {
    this->leg_position.X = position.X;
    this->leg_position.Y = position.Y;
    this->leg_position.Z = position.Z;
    return ERR_NONE;
}

uint8_t Leg::set_leg_position(position_t position) {
    position_t new_position = {position.X, position.Y, position.Z}
    new_position.X -= this->offset.X;
    new_position.Y -= this->offset.Y;

    uint16_t r = sqrt(new_position.X**2, new_position.Y**2);
    uint16_t theta = atan2(new_position.Y, new_position.X);
    theta -= deg2rad(this->offset.rotation);
    
    new_position.X = cos(theta) * r;
    new_position.Y = sin(theta) * r;

    this->leg_position.X = new_position.X;
    this->leg_position.Y = new_position.Y;
    this->leg_position.Z = new_position.Z;
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
    int16_t result = atan2(this->leg_position.X, this->leg_position.Y);

    if (result > (M_PI)) {
        result -= 2 * M_PI;
    }

    this->coxa_current_position = rad2deg(result);
    return ERR_NONE;
}

uint8_t Leg::calculate_femur_position() {
    int16_t new_x = sqrt(this->leg_position.X**2 + this->leg_position.Y**2);
    new_x -= this->coxa_length;

    int16_t new_z = this->leg_position.Z;
    int16_t hypotenuse = sqrt(new_x**2, new_z**2);

    int32_t first_part = (hypotenuse**2 + this->femur_length**2 - this->tibia_length**2);
    int32_t second_part = (2 * hypotenuse * this->femur_length);
    float division = first_part / second_part;
    float B = acos(division);

    float A = atan2(new_z, new_x);

    if (A > M_PI) {
        A -= 2 * M_PI;
    }

    float result = B - A;

    this->femur_current_position = rad2deg(result);

    return ERR_NONE;
}

uint8_t Leg::calculate_tibia_position() {
    int16_t new_x = sqrt(this->leg_position.X**2 + this->leg_position.Y**2);
    new_x -= this->coxa_length;

    int16_t new_z = this->leg_position.Z;
    int16_t hypotenuse = sqrt(new_x**2, new_z**2);

    int32_t first_part = (this->femur_length**2 + this->tibia_length**2 - hypotenuse**2);
    int32_t second_part = (2 * hypotenuse * this->femur_length);
    float division = first_part / second_part;
    float result = M_PI - acos(division);

    this->tibia_current_position = rad2deg(result);

    return ERR_NONE;
}

uint8_t Leg::calculate_positions() {
    this->calculate_coxa_position();
    this->calculate_tibia_position();
    this->calculate_femur_position();

    return ERR_NONE;
}

uint8_t Leg::move_coxa() const {
    return ERR_NONE;
}

uint8_t Leg::move_femur() const {
    return ERR_NONE;
}

uint8_t Leg::move_tibia() const {
    return ERR_NONE;
}

uint8_t Leg::move_leg() const {
    return ERR_NONE;
}