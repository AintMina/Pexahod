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

// System
#include "pico/stdlib.h"
#include <math.h>

// Servo2040
#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"


// Local
#include "led.h"
#include "nrf.h"
#include "queues.h"
#include "mode_handler.h"

using namespace servo;


int main() {
    // NEEDED FOR DEBUG!!!
    timer_hw->dbgpause = 0;

    stdio_init_all();

    // Removes automatic 0x0d when sending 0x0a
    stdio_set_translate_crlf(&stdio_usb, false);
    sleep_ms(10);

    init_queue();   // Init RTOS queues
	  led_init();     // Init LED HW
    init_mode();    // Init tasks and mode

    // Start the scheduler
    vTaskStartScheduler();

    while (1) {

    }

    return 0;
}