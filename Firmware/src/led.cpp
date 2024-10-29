#include "led.h"
#include "servo2040.hpp"

// FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"


using namespace plasma;
using namespace servo;

// Create the LED bar, using PIO 1 and State Machine 0
WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);
static SemaphoreHandle_t led_mutex;


void led_init() {
    // Start updating the LED bar
    led_bar.start();

    // Create mutex
    led_mutex = xSemaphoreCreateMutex();
    if (led_mutex == NULL) {
        // Handle error: mutex creation failed
        // You might log an error or stop execution, depending on the application
    }
}

void led_clear() {
    // Turn off the LED bar
    led_bar.clear();
}

void set_led(uint8_t led, uint8_t r, uint8_t g, uint8_t b) {
    if (xSemaphoreTake(led_mutex, portMAX_DELAY) == pdTRUE) {
        // Critical section - safely modify the LED
        led_bar.set_rgb(led, r, g, b);

        // Give the mutex back when done
        xSemaphoreGive(led_mutex);
    } 
    else {
        // Handle case where the mutex could not be taken
        // (e.g., timeout or an error, depending on your use case)
    }
}