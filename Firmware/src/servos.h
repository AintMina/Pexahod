#ifndef SERVOS_H
#define SERVOS_H



#include "pico/stdlib.h"
#include "servo2040.hpp"
using namespace servo;

#include "leg.hpp"


uint8_t init_servos();
uint8_t update_servos(Leg *legs[]);
uint8_t set_servo(uint8_t index, float value);



#endif