#include "main.h"
#include "hw_init.h"

#include "com/datalink.h"
#include "com/debug.h"
#include "com/com_fileio.h"
#include "com/com_iface.h"
#include "bus/event_queue.h"
#include "bus/event_handler.h"
#include "utils/timebase.h"
#include "utils/debounce.h"

#include <math.h>
#include <sbmp.h>

#include "dspin.h"
#include "blinky.h"

// 7.5deg motor -> 48 full steps. We use 128 step mode -> 6144 = full circle
#define STEPS_360 6144

static void poll_subsystems(void);
static void conf_buttons(void);
static void try_sbmp_hsk(void *unused);

/** Main program function */
int main(void)
{
	hw_init();
	conf_buttons();

	banner("*** STM32F105 stepper motor demo ***");
	banner_info("(c) Ondrej Hruska, 2016");
	banner_info("Katedra mereni K338, CVUT FEL");

	// Intro animation
	led_blink(LED_SPARE, 200);
	delay_ms(100);
	led_blink(LED_ERROR, 200);
	delay_ms(100);
	led_blink(LED_BUSY, 200);
	delay_ms(100);
	led_toggle(LED_READY);

	// Green LED starts flashing...

	add_periodic_task(try_sbmp_hsk, NULL, 500, true);

	ms_time_t t = ms_now();
	while (1) {
		poll_subsystems();

		// blink to indicate we're working OK
		if (ms_loop_elapsed(&t, 500)) {
			led_toggle(LED_READY);
		}
	}
}

static void try_sbmp_hsk(void *unused)
{
	(void)unused;

	if (sbmp_ep_handshake_status(dlnk_ep) != SBMP_HSK_SUCCESS) {
		sbmp_ep_start_handshake(dlnk_ep);
	}
}

/** Left button was pressed */
static void left_btn_down(void)
{
	dSPIN_Run(REV, STEPS_360*2);
}

/** Right button was pressed */
static void right_btn_down(void)
{
	dSPIN_Run(FWD, STEPS_360*2);
}

/** Left button was pressed */
static void left_btn_up(void)
{
	dSPIN_Soft_Stop();
}

/** Right button was pressed */
static void right_btn_up(void)
{
	dSPIN_Soft_Stop();
}

/** Datagram rx on SBMP (extern declated in datalink.h) */
void dlnk_rx(SBMP_Datagram *dg)
{
	(void)dg;

	PayloadParser pp = pp_start(dg->payload, dg->length);

	int32_t speed;
	uint8_t fwd;

	led_blink(LED_BUSY, 50);

	switch (dg->type) {
		case DG_MOTOR_START:
			fwd = (bool) pp_u8(&pp);
			speed = pp_u32(&pp);

			if (speed > 6144*3) {
				speed = 6144*3;
			}

			dSPIN_Run(fwd ? FWD : REV, speed);
			break;

		case DG_MOTOR_STOP:
			dSPIN_Soft_Stop();
			break;
	}
}

/** Configure debouncer and button callbacks */
static void conf_buttons(void)
{
	debounce_init(2);

	// setup debouncer
	debo_init_t debo = {};
	debo.debo_time = 10;
	debo.invert = true;

	// button A
	debo.GPIOx = BUTTON_A_Port;
	debo.pin = BUTTON_A_Pin;
	debo.rising_cb = left_btn_down;
	debo.falling_cb = left_btn_up;
	debo_register_pin(&debo);

	// Button B
	debo.GPIOx = BUTTON_B_Port;
	debo.pin = BUTTON_B_Pin;
	debo.rising_cb = right_btn_down;
	debo.falling_cb = right_btn_up;
	debo_register_pin(&debo);

	add_periodic_task(debo_periodic_task, NULL, 10, true);
}

/** Poll subsystems, called in the main loop */
static void poll_subsystems(void)
{
	// poll serial buffers (runs callback)
	com_poll(debug_iface);
	com_poll(data_iface);

	// run queued tasks
	tq_poll();

	// handle queued events
	Event evt;

	until_timeout(2) { // take 2 ms max
		if (eq_take(&evt)) {
			run_event_handler(&evt);
		} else {
			break;
		}
	}
}
