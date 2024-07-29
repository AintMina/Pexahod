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

// servo2040
#include "servo2040.hpp"
#include "analogmux.hpp"
#include "analog.hpp"
#include "button.hpp"

using namespace plasma;
using namespace servo;

// The speed that the LEDs will cycle at
const uint SPEED = 5;

// The brightness of the LEDs
constexpr float BRIGHTNESS = 0.4f;

// How many times the LEDs will be updated per second
const uint UPDATES = 50;


// Create the LED bar, using PIO 1 and State Machine 0
WS2812 led_bar(servo2040::NUM_LEDS, pio1, 0, servo2040::LED_DATA);

// Create the user button
Button user_sw(servo2040::USER_SW);


#define LED_PIN 26

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;

rcl_timer_t timer;
rcl_node_t node;
rcl_allocator_t allocator;
rclc_support_t support;
rclc_executor_t executor;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time) {
    rcl_ret_t ret = rcl_publish(&publisher, &msg, NULL);
    msg.data++;
}

void vBlinkTask(void *pvParameters) {

    for (;;) {
        gpio_put(LED_PIN, 1);
        vTaskDelay(250);
        gpio_put(LED_PIN, 0);
        vTaskDelay(250);
    }

}

void ros_command_task(void *pvParameters) {
    while (1) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
}

int main() {
    stdio_init_all();

    // Create a servo cluster for pins 0 to 3, using PIO 0 and State Machine 0
    const uint START_PIN = servo2040::SERVO_1;
    const uint END_PIN = servo2040::SERVO_18;
    const uint NUM_SERVOS = (END_PIN - START_PIN) + 1;
    Calibration ecal = Calibration();
    // ecal.apply_two_pairs(1000, 2500, -(270/2), 270/2);
    ecal.apply_three_pairs(1000, 1500, 2500, -(270/2), 0, (270/2));
    ServoCluster servos = ServoCluster(pio0, 0, START_PIN, NUM_SERVOS, ecal);

    // Initialise the servo cluster
    servos.init();
    servos.enable_all();
    sleep_ms(10000);

    servos.all_to_value(0);
    sleep_ms(2000);
    servos.all_to_value(90);
    sleep_ms(4000);
    servos.all_to_value(-90);
    sleep_ms(4000);
    servos.all_to_mid();
    sleep_ms(2000);

    servos.disable_all();
    sleep_ms(2000);


    // Create an empty calibration;
    // Servo tibia1 = Servo(servo2040::SERVO_3, ecal);

    // // Initialise the servo cluster
    // tibia1.init();
    // tibia1.value(0);
    // sleep_ms(10000);
    // tibia1.disable();

    // Start updating the LED bar
    led_bar.start();


    rmw_uros_set_custom_transport(
		true,
		NULL,
		pico_serial_transport_open,
		pico_serial_transport_close,
		pico_serial_transport_write,
		pico_serial_transport_read
	);

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);

    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);

    rclc_node_init_default(&node, "pico_node", "", &support);
    rclc_publisher_init_default(
        &publisher,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
        "pico_publisher");

    rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback,
        1);

    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);


    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);

    xTaskCreate(vBlinkTask, "Blink Task", 128, NULL, 1, NULL);
    xTaskCreate(ros_command_task, "ROS Task", 128, NULL, 1, NULL);
    vTaskStartScheduler();
}