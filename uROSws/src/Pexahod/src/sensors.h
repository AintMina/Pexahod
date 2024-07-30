#ifndef SENSORS_H
#define SENSORS_H



#include <cstdio>
#include "pico/stdlib.h"

#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"
#include "button.hpp"
using namespace servo;


uint8_t sensors_init();
uint8_t read_end_Stop_sensors();
float read_voltage_sensor();
float read_current_sensor();



#endif