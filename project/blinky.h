#ifndef BLINKY_H
#define BLINKY_H
#include "utils/timebase.h"

enum led_nr {
	LED_READY,
	LED_BUSY,
	LED_ERROR,
	LED_SPARE
};

void led_on(enum led_nr led);
void led_off(enum led_nr led);
void led_toggle(enum led_nr led);
void led_blink(enum led_nr led, ms_time_t ms);

#endif // BLINKY_H
