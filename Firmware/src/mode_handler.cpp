// Local
#include "mode_handler.h"
#include "robot_controller.h"
#include "usb.h"
#include "sensors.h"
#include "command_handler.h"
#include "nrf.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"



static TaskHandle_t xRobotControllerHandle = NULL;
static TaskHandle_t xNrfHandle = NULL;
static TaskHandle_t xUsbHandle = NULL;
static TaskHandle_t xCommandHandle = NULL;
static TaskHandle_t xSensorHandle = NULL;

static enum mode_e mode = READY_MODE;

enum mode_e get_mode() {
    return mode;
}

void set_mode(enum mode_e new_mode) {
    mode = new_mode;
}

void init_mode() {
    // Create tasks
    xTaskCreate(robot_controller_main, "RobotControllerTask", 2048, NULL, 1, &xRobotControllerHandle);
    xTaskCreate(usb_main, "UsbTask", 1024, NULL, 1, &xUsbHandle);
    xTaskCreate(sensors_main, "SensorTask", 1024, NULL, 2, &xSensorHandle);
    xTaskCreate(command_handler_main, "CommandTask", 2048, NULL, 2, &xCommandHandle);
    xTaskCreate(nrf_main, "NrfTask", 1024, NULL, 1, &xNrfHandle);
	
    // // Restrict to cores
    // UBaseType_t uxCoreAffinityMask;
    // uxCoreAffinityMask = (( 1 << 0 ));
    // vTaskCoreAffinitySet(xRobotControllerHandle, uxCoreAffinityMask);

    // uxCoreAffinityMask = (( 1 << 1 ));
    // vTaskCoreAffinitySet(xNrfHandle, uxCoreAffinityMask);
    // vTaskCoreAffinitySet(xUsbHandle, uxCoreAffinityMask);
    // vTaskCoreAffinitySet(xSensorHandle, uxCoreAffinityMask);
    // vTaskCoreAffinitySet(xCommandHandle, uxCoreAffinityMask);

    switch (mode) {
        case CALIBRATION_MODE:
            /* code */
            break;
        case IDLE_MODE:
            /* code */
            vTaskSuspend(xRobotControllerHandle);
            break;
        case READY_MODE:
            /* code */
            vTaskResume(xRobotControllerHandle);
            break;
        
        default:
            break;
    }

}