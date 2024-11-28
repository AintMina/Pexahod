#ifndef MODE_HANDLER_H
#define MODE_HANDLER_H



#include <stdint.h>


enum mode_e : uint8_t {
    CALIBRATION_MODE,
    IDLE_MODE,
    READY_MODE
};


enum mode_e get_mode();
void init_mode();
void set_mode(enum mode_e new_mode);



#endif // MODE_HANDLER_H