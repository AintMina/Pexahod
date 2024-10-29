#ifndef LED_H
#define LED_H



#include <stdint.h>

void led_init();
void led_clear();
void set_led(uint8_t led, uint8_t r, uint8_t g, uint8_t b);



#endif // LED_H