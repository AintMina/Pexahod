#include "unit_conversion.h"

#include <math.h>
#include <string.h>



double deg_to_rad(double degrees) {
    return degrees * PI_OVER_180;
}

double rad_to_deg(double radians) {
    return radians * PI_UNDER_180;
}

// Converts a float to a uint8_t array
void float_to_uint(float value, uint8_t buffer[sizeof(float)]) {
    memcpy(buffer, &value, sizeof(float));
}

// Converts a uint8_t array back to a float
float uint_to_float(uint8_t buffer[sizeof(float)]) {
    float value;
    memcpy(&value, buffer, sizeof(float));
    return value;
}