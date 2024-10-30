#include "usb.h"
#include "nrf_msg.h"
#include "queues.h"

#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <tusb.h>


const uint16_t usb_delay = 1;

void usb_init() {

}

void read_custom_message() {
    if (tud_cdc_connected()) {
        struct nrf_message_t message;
        uint8_t *message_ptr = (uint8_t *)&message;
        memset(&message, 0, sizeof(message));

        while (tud_cdc_available()) {
            char read = getchar();
            *message_ptr = read;

            if (message.prefix != PREFIX) {
                message_ptr = (uint8_t *)&message;
            }

            message_ptr++;
            if (message.command && message.command < COMMAND_ENUM_SIZE) {
                if (message_ptr >= ((uint8_t *)&message + (sizeof(message) - sizeof(message.data)) + message.length)) {
                    send_to_queue(0, &message, 1);
                    memset(&message, 0, sizeof(message));
                    message_ptr = (uint8_t *)&message;
                }
            }

        }
    }
}

void send_custom_message(struct nrf_message_t *msg) {
    printf("%c", msg->prefix);
    printf("%c", msg->length);
    printf("%c", msg->id);
    printf("%c", msg->command);
    for (int i = 0; i < msg->length; i++) {
        printf("%c", msg->data[i]);
    }
    printf("%c", msg->crc);
}

void usb_main(void *pvParameters) {
    while (1) {
        read_custom_message();

        // Send command
        struct nrf_message_t message;
        BaseType_t ret = receive_from_queue(1, &message, 1);

        if (ret == pdPASS) {
            send_custom_message(&message);
        }

        vTaskDelay(usb_delay / portTICK_PERIOD_MS);
    }
}