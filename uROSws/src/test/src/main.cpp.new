#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <std_msgs/msg/int32.h>
#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "pico_uart_transports.h"

#define LED_PIN 26

void vBlinkTask() {

   for (;;) {
      gpio_put(LED_PIN, 1);
      vTaskDelay(250);
      gpio_put(LED_PIN, 0);
      vTaskDelay(250);
   }

}

void main() {
   gpio_init(LED_PIN);
   gpio_set_dir(LED_PIN, GPIO_OUT);
   xTaskCreate(vBlinkTask, "Blink Task", 128, NULL, 1, NULL);
   vTaskStartScheduler();
}