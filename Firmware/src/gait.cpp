#include "gait.hpp"
#include "bezier.h"
#include "leg.hpp"
#include <math.h>


struct gait_t trigait = {
    .t_offsets = {0.0, 0.5, 0.0, 0.5, 0.0, 0.5},
    .t_in_air = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5}
};

struct gait_t wavegait = {
    .t_offsets = {0.0, (1.0/6.0), (2.0/6.0), (3.0/6.0), (4.0/6.0), (5.0/6.0)},
    .t_in_air = {1.0/6.0, 1.0/6.0, 1.0/6.0, 1.0/6.0, 1.0/6.0, 1.0/6.0}
};

struct gait_t ripplegait = {
    .t_offsets = {0.0, 1.0/3.0, 2.0/3.0, 1.0/6.0, 3.0/6.0, 5.0/6.0},
    .t_in_air = {1.0/3.0, 1.0/3.0, 1.0/3.0, 1.0/3.0, 1.0/3.0, 1.0/3.0}
};

struct gait_t gaits[NUM_GAITS] = {trigait, wavegait, ripplegait};


void get_gait_point_v1(position_t return_points[6], Leg *legs[6], float time) {
    for (int i = 0; i < 6; i++) {
        enum Gaits gait = legs[i]->get_gait();
        float t_in_air_end = gaits[gait].t_offsets[i] + gaits[gait].t_in_air[i];

        uint8_t status = 0;
        if (gaits[gait].t_offsets[i] <= time && time <= t_in_air_end) {
            status = 1;
        }
        else if (t_in_air_end > 1.0 && time <= t_in_air_end - 1.0) {
            status = 2;
        }
        else if (t_in_air_end < 1.0 && time > t_in_air_end) {
            status = 3;
        }
        else {
            status = 4;
        }

        int32_t point[2];
        switch (status) {
            case 1: {
                // In air
                float t = (time - gaits[gait].t_offsets[i]) / gaits[gait].t_in_air[i];

                if (t > 1.0) {
                    t = 1.0;
                }

                int32_t step_size = legs[i]->get_max_step_size() / 2.0;
                int32_t start_point[2] = {-step_size, 0};
                int32_t end_point[2] = {step_size, 0};
                get_bezier_point(point, start_point, end_point, t);
                break;
            }
            case 2: {
                // In air
                float t = ((time + 1.0) - gaits[gait].t_offsets[i]) / gaits[gait].t_in_air[i];

                if (t > 1.0) {
                    t = 1.0;
                }

                int32_t step_size = legs[i]->get_max_step_size() / 2.0;
                int32_t start_point[2] = {-step_size, 0};
                int32_t end_point[2] = {step_size, 0};
                get_bezier_point(point, start_point, end_point, t);
                break;
            }
            case 3: {
                // On ground
                float t = (time - gaits[gait].t_offsets[i] - gaits[gait].t_in_air[i]) / (1.0 - gaits[gait].t_in_air[i]);

                if (t > 1.0) {
                    t = 1.0;
                }

                int32_t step_size = legs[i]->get_max_step_size() / 2.0;
                int32_t start_point[2] = {-step_size, 0};
                int32_t end_point[2] = {step_size, 0};
                get_bezier_line_point(point, end_point, start_point, t);
                break;
            }
            case 4: {
                // On ground
                float t = ((time + 1.0) - gaits[gait].t_offsets[i] - gaits[gait].t_in_air[i]) / (1.0 - gaits[gait].t_in_air[i]);

                if (t > 1.0) {
                    t = 1.0;
                }

                int32_t step_size = legs[i]->get_max_step_size() / 2.0;
                int32_t start_point[2] = {-step_size, 0};
                int32_t end_point[2] = {step_size, 0};
                get_bezier_line_point(point, end_point, start_point, t);
                break;
            }
        }

        position_t offset;
        legs[i]->get_gait_offset(&offset);

        return_points[i].X = offset.X;
        return_points[i].Y = point[0] + offset.Y;
        return_points[i].Z = point[1] + offset.Z;
    }
}


void get_gait_point_v2(position_t return_points[6], Leg *legs[6], float time, velocity_t vel) {
    for (int i = 0; i < 6; i++) {
        enum Gaits gait = legs[i]->get_gait();
        float t_in_air_end = gaits[gait].t_offsets[i] + gaits[gait].t_in_air[i];

        uint8_t status = 0;
        if (gaits[gait].t_offsets[i] <= time && time <= t_in_air_end) {
            status = 1;
        }
        else if (t_in_air_end > 1.0 && time <= t_in_air_end - 1.0) {
            status = 2;
        }
        else if (t_in_air_end < 1.0 && time > t_in_air_end) {
            status = 3;
        }
        else {
            status = 4;
        }

        int32_t point[2];
        int32_t point2[2];
        float step_size = legs[i]->get_max_step_size() / 2.0f;
        int32_t start_point[2] = {static_cast<int32_t>(-step_size * vel.Y), 0};
        int32_t end_point[2] = {static_cast<int32_t>(step_size * vel.Y), 0};
        int32_t start_point2[2] = {static_cast<int32_t>(-step_size * vel.X), 0};
        int32_t end_point2[2] = {static_cast<int32_t>(step_size * vel.X), 0};
        switch (status) {
            case 1: {
                // In air
                float t = (time - gaits[gait].t_offsets[i]) / gaits[gait].t_in_air[i];

                if (t > 1.0) {
                    t = 1.0;
                }

                get_bezier_point(point, start_point, end_point, t);
                get_bezier_point(point2, start_point2, end_point2, t);
                break;
            }
            case 2: {
                // In air
                float t = ((time + 1.0) - gaits[gait].t_offsets[i]) / gaits[gait].t_in_air[i];

                if (t > 1.0) {
                    t = 1.0;
                }

                get_bezier_point(point, start_point, end_point, t);
                get_bezier_point(point2, start_point2, end_point2, t);
                break;
            }
            case 3: {
                // On ground
                float t = (time - gaits[gait].t_offsets[i] - gaits[gait].t_in_air[i]) / (1.0 - gaits[gait].t_in_air[i]);

                if (t > 1.0) {
                    t = 1.0;
                }

                get_bezier_line_point(point, end_point, start_point, t);
                get_bezier_line_point(point2, end_point2, start_point2, t);
                break;
            }
            case 4: {
                // On ground
                float t = ((time + 1.0) - gaits[gait].t_offsets[i] - gaits[gait].t_in_air[i]) / (1.0 - gaits[gait].t_in_air[i]);

                if (t > 1.0) {
                    t = 1.0;
                }

                get_bezier_line_point(point, end_point, start_point, t);
                get_bezier_line_point(point2, end_point2, start_point2, t);
                break;
            }
        }

        position_t offset;
        legs[i]->get_gait_offset(&offset);

        return_points[i].X = point2[0] + offset.X;
        return_points[i].Y = point[0] + offset.Y;
        return_points[i].Z = ((point[1] + point2[1]) / 2) + offset.Z;
    }
}