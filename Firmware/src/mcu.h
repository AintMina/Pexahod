#ifndef MCU_H
#define MCU_H


#include <stdint.h>


#define SMART_MODE 1
#define DUMB_MODE 0

uint8_t get_mode();
void set_mode(uint8_t new_mode);
void toggle_mode();
void init_mode();


#endif // MCU_H