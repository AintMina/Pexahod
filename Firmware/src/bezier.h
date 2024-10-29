#ifndef BEZIER_H
#define BEZIER_H


#include <stdint.h>

void get_bezier_middle_point(int32_t *return_point, int32_t *start_point, int32_t *end_point);
void get_bezier_point(int32_t *return_point, int32_t *start_point, int32_t *end_point, float time);
void get_bezier_point(int32_t *return_point, int32_t *start_point, int32_t *middle_point, int32_t *end_point, float time);
void get_bezier_line_point(int32_t *return_point, int32_t *start_point, int32_t *end_point, float time);


#endif // BEZIER_H