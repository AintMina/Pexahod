#include "mcu.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"


static uint8_t mode = DUMB_MODE;

uint8_t get_mode() {
    return mode;
}

void set_mode(uint8_t new_mode) {
    mode = new_mode;
}

void toggle_mode() {
    if (mode == DUMB_MODE) {
        mode = SMART_MODE;
    }
    else {
        mode = DUMB_MODE;
    }
}

void init_mode() {

}