#include "main.h"
#include "utils/timebase.h"
#include "blinky.h"
#include "dspin.h"

typedef struct {
	GPIO_TypeDef* GPIOx;
	uint32_t pin;
	task_pid_t blink_task;
} LedInfo;

static LedInfo leds[4] = {
	{GPIOC, GPIO_Pin_3, 0}, // LED_READY
	{GPIOC, GPIO_Pin_2, 0}, // LED_BUSY
	{GPIOC, GPIO_Pin_1, 0}, // LED_ERROR
	{GPIOC, GPIO_Pin_0, 0}, // LED_SPARE
};

void led_on(enum led_nr led)
{
	leds[led].GPIOx->BSRR = leds[led].pin;
}

void led_off(enum led_nr led)
{
	leds[led].GPIOx->BRR = leds[led].pin;
}

void led_toggle(enum led_nr led)
{
	leds[led].GPIOx->ODR ^= leds[led].pin;
}

static void led_off_cb(void *arg)
{
	enum led_nr led = (uint32_t)arg;

	led_off(led);
	leds[led].blink_task = 0;
}

void led_blink(enum led_nr led, ms_time_t ms)
{
	LedInfo *L = &leds[led];

	if (L->blink_task) {
		// re-schedule
		abort_scheduled_task(L->blink_task);
	}

	led_on(led);
	L->blink_task = schedule_task(led_off_cb, (uint32_t*)led, ms, false);
}
