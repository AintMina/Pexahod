#ifndef UNIT_CENVERSION_H
#define UNIT_CENVERSION_H



#include <stdint.h>


#define PI 3.14159
#define PI_OVER_180 0.01745329251
#define PI_UNDER_180 57.2957795131
#define PI_OVER_2 1.57079632679


double deg_to_rad(double degrees);
double rad_to_deg(double radians);
void float_to_uint(float value, uint8_t buffer[sizeof(float)]);
float uint_to_float(uint8_t buffer[sizeof(float)]);



#endif // UNIT_CENVERSION_H