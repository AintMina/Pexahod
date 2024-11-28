#ifndef GAIT_HPP
#define GAIT_HPP

/*

     (1)               (6)
       \               /
        \             /
         \           /
          *---(F)---*
         /           \
        /             \
       /               \
(2)---*                 *---(5)
       \               /
        \             /
         \           /
          *---(B)---*
         /           \
        /             \
       /               \
     (3)               (4)

*/

#include "robot.h"


struct gait_t {
    float t_offsets[6];
    float t_in_air[6];
};

enum gaits_e {
    TRIGAIT,
    WAVEGAIT,
    RIPPLEGAIT,
    NUM_GAITS // Needs to be last!
};

class Leg;
void get_gait_point_v1(position_t return_points[6], Leg *legs[6], float time);
void get_gait_point_v2(position_t return_points[6], Leg *legs[6], float time, velocity_t vel);


#endif // GAIT_HPP