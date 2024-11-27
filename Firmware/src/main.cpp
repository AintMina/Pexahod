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

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"


// Local
#include "robot_controller.hpp"
#include "led.h"
#include "nrf.h"
#include "queues.h"
#include "usb.h"
#include "nrf_msg.h"
#include "command_handler.h"
#include "sensors.h"

using namespace servo;

TaskHandle_t xRobotControllerHandle = NULL;
TaskHandle_t xNrfHandle = NULL;
TaskHandle_t xUsbHandle = NULL;
TaskHandle_t xCommandHandle = NULL;
TaskHandle_t xSensorHandle = NULL;

TaskHandle_t xTempHandle = NULL;


int main() {
    // NEEDED FOR DEBUG!!!
    timer_hw->dbgpause = 0;

    stdio_init_all();

    // Remove automatic 0x0d when sending 0x0a
    stdio_set_translate_crlf(&stdio_usb, false);
    sleep_ms(10);

    init_queue();
	  led_init();

    xTaskCreate(robot_controller_main, "RobotControllerTask", 2048, NULL, 1, &xRobotControllerHandle);
    xTaskCreate(usb_main, "UsbTask", 1024, NULL, 1, &xUsbHandle);
    xTaskCreate(sensors_main, "SensorTask", 1024, NULL, 2, &xSensorHandle);
    xTaskCreate(command_handler_main, "CommandTask", 2048, NULL, 2, &xCommandHandle);
    //xTaskCreate(nrf_main, "NrfTask", 1024, NULL, 1, &xNrfHandle);
	
    // UBaseType_t uxCoreAffinityMask;
    // uxCoreAffinityMask = (( 1 << 0 ));
    // vTaskCoreAffinitySet(xRobotControllerHandle, uxCoreAffinityMask);

    // uxCoreAffinityMask = (( 1 << 1 ));
    // vTaskCoreAffinitySet(xNrfHandle, uxCoreAffinityMask);
    // vTaskCoreAffinitySet(xUsbHandle, uxCoreAffinityMask);
    // vTaskCoreAffinitySet(xSensorHandle, uxCoreAffinityMask);
    // vTaskCoreAffinitySet(xCommandHandle, uxCoreAffinityMask);

    // Start the scheduler
    vTaskStartScheduler();

    while (1) {

    }

    return 0;
}