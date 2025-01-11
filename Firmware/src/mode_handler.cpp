// Local
#include "mode_handler.h"
#include "robot_controller.h"
#include "usb.h"
#include "sensors.h"
#include "command_handler.h"
#include "nrf.h"
#include "calibration.h"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


static TaskHandle_t xRobotControllerHandle = NULL;
static TaskHandle_t xCalibrationHandle = NULL;
static TaskHandle_t xNrfHandle = NULL;
static TaskHandle_t xUsbHandle = NULL;
static TaskHandle_t xCommandHandle = NULL;
static TaskHandle_t xSensorHandle = NULL;

static enum mode_e mode = READY_MODE;
static SemaphoreHandle_t mode_mutex;

enum mode_e get_mode() {
    return mode;
}

void set_mode(enum mode_e new_mode) {
    if (new_mode >= NR_MODES) {
        return;
    }

    if (xSemaphoreTake(mode_mutex, portMAX_DELAY) == pdTRUE) {
        mode = new_mode;

        switch (mode) {
            case CALIBRATION_MODE:
                /* code */
                vTaskResume(xCalibrationHandle);
                vTaskSuspend(xRobotControllerHandle);
                vTaskSuspend(xSensorHandle);
                break;
            case IDLE_MODE:
                /* code */
                vTaskSuspend(xRobotControllerHandle);
                vTaskSuspend(xCalibrationHandle);
                vTaskSuspend(xSensorHandle);
                break;
            case READY_MODE:
                /* code */
                vTaskResume(xRobotControllerHandle);
                vTaskSuspend(xCalibrationHandle);
                vTaskSuspend(xSensorHandle);
                break;
            
            default:
                break;
        }

        // Give the mutex back when done
        xSemaphoreGive(mode_mutex);
    }
}

void init_mode() {
    // Create a mutex for this instance
    mode_mutex = xSemaphoreCreateMutex();
    if (mode_mutex == NULL) {
        // Handle error: failed to create mutex
    }

    // Create tasks
    xTaskCreate(robot_controller_main, "RobotControllerTask", 2048, NULL, 1, &xRobotControllerHandle);
    xTaskCreate(calibration_main, "CalibrationTask", 1024, NULL, 1, &xCalibrationHandle);
    xTaskCreate(usb_main, "UsbTask", 1024, NULL, 1, &xUsbHandle);
    xTaskCreate(sensors_main, "SensorTask", 1024, NULL, 2, &xSensorHandle);
    xTaskCreate(command_handler_main, "CommandTask", 1024, NULL, 2, &xCommandHandle);
    //xTaskCreate(nrf_main, "NrfTask", 1024, NULL, 1, &xNrfHandle);

    // // Restrict to cores
    // UBaseType_t uxCoreAffinityMask;
    // uxCoreAffinityMask = (( 1 << 0 ));
    // vTaskCoreAffinitySet(xRobotControllerHandle, uxCoreAffinityMask);

    // uxCoreAffinityMask = (( 1 << 1 ));
    // vTaskCoreAffinitySet(xNrfHandle, uxCoreAffinityMask);
    // vTaskCoreAffinitySet(xUsbHandle, uxCoreAffinityMask);
    // vTaskCoreAffinitySet(xSensorHandle, uxCoreAffinityMask);
    // vTaskCoreAffinitySet(xCommandHandle, uxCoreAffinityMask);

    set_mode(mode);
}