#pragma once

/**
 * SBMP setup & funcs
 */

#include "main.h"
#include <sbmp.h>

#define DG_MOTOR_START 30
#define DG_MOTOR_STOP 31

// wifi status & control
#define DG_SETMODE_AP 44 // request AP mode (AP button pressed)
#define DG_WPS_START 45 // start WPS
#define DG_WIFI_STATUS 46 // WiFi status report (sent by the ESP every second)
#define DG_REQUEST_STM_VERSION 47 // Get STM32 module firmware version


extern SBMP_Endpoint *dlnk_ep;

void dlnk_init(void);

/** Received datagram handler */
extern void dlnk_rx(SBMP_Datagram *dg);
