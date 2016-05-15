#pragma once

/**
 * NOOP iface works like /dev/null
 */

#include "com_iface.h"

/**
 * @brief Initialize a do-nothing interface.
 * @param iface : iface pointer. If NULL, it'll be allocated.
 * @return the iface pointer.
 */
ComIface *noop_iface_init(void);
