#include "leg.hpp"
#include "errors.h"

// Constructors
Leg::Leg(offset_t offset, uint coxa, uint femur, uint tibia) {}
Leg::Leg(offset_t offset, int16_t coxa_length, int16_t femur_length, int16_t tibia_length, uint coxa, uint femur, uint tibia) {}

uint8_t Leg::init() {
    return 0;
}

uint8_t Leg::enable() {
    this->enabled = 1;
    return 0;
}

uint8_t Leg::disable() {
    this->enabled = 0;
    return 0;
}

uint8_t Leg::is_enabled() const {
    return this->enabled;
}

uint8_t Leg::set_offset(offset_t offset) {
    this->offset.X = offset.X;
    this->offset.Y = offset.Y;
    this->offset.rotation = offset.rotation;
    return 0;
}

uint8_t Leg::get_offset(offset_t offset) const {
    offset.X = this->offset.X;
    offset.Y = this->offset.Y;
    offset.rotation = this->offset.rotation;
    return 0;
}

uint8_t Leg::set_coxa_length(uint16_t length) {
    this->coxa_length = length;
    return 0;
}

uint16_t Leg::get_coxa_length() const {
    return this->coxa_length;
}

uint8_t Leg::set_femur_length(uint16_t length) {
    this->femur_length = length;
    return 0;
}

uint16_t Leg::get_femur_length() const {
    return this->femur_length;
}

uint8_t Leg::set_tibia_length(uint16_t length) {
    this->tibia_length = length;
    return 0;
}

uint16_t Leg::get_tibia_length() const {
    return this->tibia_length;
}

uint8_t Leg::set_coxa_position(int16_t position) {
    this->coxa_current_position = position;
    return 0;
}

int16_t Leg::get_coxa_position() const {
    return this->coxa_current_position;
}

uint8_t Leg::set_femur_position(int16_t position) {
    this->femur_current_position = position;
    return 0;
}

int16_t Leg::get_femur_position() const {
    return this->femur_current_position;
}

uint8_t Leg::set_tibia_position(int16_t position) {
    this->tibia_current_position = position;
    return 0;
}

int16_t Leg::get_tibia_position() const {
    return this->tibia_current_position;
}

uint8_t Leg::set_coxa_limits(int16_t min, int16_t max) {
    this->coxa_limits[0] = min;
    this->coxa_limits[1] = max;
    return 0;
}

uint8_t Leg::set_femur_limits(int16_t min, int16_t max) {
    this->femur_limits[0] = min;
    this->femur_limits[1] = max;
    return 0;
}

uint8_t Leg::set_tibia_limits(int16_t min, int16_t max) {
    this->tibia_limits[0] = min;
    this->tibia_limits[1] = max;
    return 0;
}

uint8_t Leg::set_leg_position(position_t position) {
    this->leg_position.X = position.X;
    this->leg_position.Y = position.Y;
    this->leg_position.Z = position.Z;
    return 0;
}

uint8_t Leg::get_leg_position(position_t position) const {
    position.X = this->leg_position.X;
    position.Y = this->leg_position.Y;
    position.Z = this->leg_position.Z;
    return 0;
}

uint8_t Leg::set_calculated_position(position_t position) {
    this->calculated_position.X = position.X;
    this->calculated_position.Y = position.Y;
    this->calculated_position.Z = position.Z;
    return 0;
}

uint8_t Leg::get_calculated_position(position_t position) const {
    position.X = this->calculated_position.X;
    position.Y = this->calculated_position.Y;
    position.Z = this->calculated_position.Z;
    return 0;
}

uint8_t Leg::set_sensor_position(int16_t *position) {
    this->sensor_position[0] = position[0];
    this->sensor_position[1] = position[1];
    this->sensor_position[2] = position[2];
    return 0;
}

uint8_t Leg::get_sensor_position(int16_t *position) const {
    position[0] = this->sensor_position[0];
    position[1] = this->sensor_position[1];
    position[2] = this->sensor_position[2];
    return 0;
}

uint8_t Leg::read_end_stop_sensor() {
    return 0;
}

uint8_t Leg::get_end_stop_sensor() const {
    return this->end_stop_sensor;
}

uint8_t Leg::transform_xyz(position_t position) {
    return 0;
}

uint8_t Leg::calculate_coxa_position() {
    return 0;
}

uint8_t Leg::calculate_femur_position() {
    return 0;
}

uint8_t Leg::calculate_tibia_position() {
    return 0;
}

uint8_t Leg::calculate_positions() {
    return 0;
}

uint8_t Leg::move_coxa() const {
    return 0;
}

uint8_t Leg::move_femur() const {
    return 0;
}

uint8_t Leg::move_tibia() const {
    return 0;
}

uint8_t Leg::move_leg() const {
    return 0;
}