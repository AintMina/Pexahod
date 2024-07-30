/*

     (1)               (6)
       \               /
        \             /
         \           /
          *---------*
         /           \
        /             \
       /               \
(2)---*                 *---(5)
       \               /
        \             /
         \           /
          *---------*
         /           \
        /             \
       /               \
     (3)               (4)

*/

// Pico
#include "pico/stdlib.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"

// uROS
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

extern "C" {
    #include "pico_uart_transports.h"
}

// Own
#include "sensors.h"
#include "servos.h"

// servo2040
#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"
#include "button.hpp"

using namespace plasma;
using namespace servo;

int main() {
    stdio_init_all();
}