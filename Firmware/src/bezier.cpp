#include "bezier.h"
#include <math.h>

#define PI 3.14159
#define PI_2 1.570796

void get_bezier_middle_point(int32_t *return_point, int32_t *start_point, int32_t *end_point) {
    float theta = atan((end_point[1] - start_point[1]) / (end_point[0] - start_point[0])) + (PI_2);
    int32_t m_point[2] = {(start_point[0] + end_point[0]) / 2, (start_point[1] + end_point[1]) / 2};
    float length = (sqrt(pow(end_point[0] - start_point[0], 2) + pow(end_point[1] - start_point[1], 2))) / 2;
    int32_t point[2] = {m_point[0] + (int32_t)(length * cos(theta)), m_point[1] + (int32_t)(length * sin(theta))};

    return_point[0] = point[0];
    return_point[1] = point[1];
}

void get_bezier_point(int32_t *return_point, int32_t *start_point, int32_t *end_point, float time) {
    int32_t middle_point[2];
    get_bezier_middle_point(middle_point, start_point, end_point);
    int32_t x = (pow(1-time, 2) * start_point[0]) + (2 * (1-time) * time * middle_point[0]) + (pow(time, 2) * end_point[0]);
    int32_t y = (pow(1-time, 2) * start_point[1]) + (2 * (1-time) * time * middle_point[1]) + (pow(time, 2) * end_point[1]);

    return_point[0] = x;
    return_point[1] = y;
}

void get_bezier_point(int32_t *return_point, int32_t *start_point, int32_t *middle_point, int32_t *end_point, float time) {
    int32_t x = (pow(1-time, 2) * start_point[0]) + (2 * (1-time) * time * middle_point[0]) + (pow(time, 2) * end_point[0]);
    int32_t y = (pow(1-time, 2) * start_point[1]) + (2 * (1-time) * time * middle_point[1]) + (pow(time, 2) * end_point[1]);
    
    return_point[0] = x;
    return_point[1] = y;
}

void get_bezier_line_point(int32_t *return_point, int32_t *start_point, int32_t *end_point, float time) {
    int32_t x = ((1-time) * start_point[0]) + ((time) * end_point[0]);
    int32_t y = ((1-time) * start_point[1]) + ((time) * end_point[1]);

    return_point[0] = x;
    return_point[1] = y;
}