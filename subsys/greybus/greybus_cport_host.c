#include <zephyr/logging/log.h>
#include "greybus_cport.h"
#include "greybus-manifest.h"

LOG_MODULE_REGISTER(greybus_cport_host, CONFIG_GREYBUS_LOG_LEVEL);

extern const struct gb_driver gb_control_host_driver;

/* Reset the counter to 0 */
enum {
	COUNTER_BASE = __COUNTER__
};
#define LOCAL_COUNTER (__COUNTER__ - COUNTER_BASE - 1)

static struct gb_cport cports[CONFIG_GREYBUS_HOST_CPORT_MAX_COUNT] = {
	/* cport0 is always control cport */
	GB_CPORT(NULL, LOCAL_COUNTER, GREYBUS_PROTOCOL_CONTROL, &gb_control_host_driver),
};

const struct gb_cport *gb_cport_get(uint16_t cport)
{
	return (cport >= CONFIG_GREYBUS_HOST_CPORT_MAX_COUNT) ? NULL : &cports[cport];
}

int gb_cports_init()
{
	return 0;
}

void gb_cports_deinit()
{
}
