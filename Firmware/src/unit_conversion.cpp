#include "unit_conversion.h"

#include <math.h>


#define PI_OVER_180 0.01745329251
#define PI_UNDER_180 57.2957795131

double deg_to_rad(double degrees) {
    return degrees * PI_OVER_180;
}

double rad_to_deg(double radians) {
    return radians * PI_UNDER_180;
}