#include <zephyr/logging/log.h>
#include "greybus_internal.h"

LOG_MODULE_REGISTER(greybus_host_control, CONFIG_GREYBUS_LOG_LEVEL);

const struct gb_driver gb_control_host_driver = {};
