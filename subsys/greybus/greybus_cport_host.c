#include <zephyr/logging/log.h>
#include "greybus_cport.h"
#include "greybus-manifest.h"
#include "greybus_heap.h"

LOG_MODULE_REGISTER(greybus_cport_host, CONFIG_GREYBUS_LOG_LEVEL);

extern const struct gb_driver gb_control_host_driver;

/* Reset the counter to 0 */
enum {
	COUNTER_BASE = __COUNTER__
};
#define LOCAL_COUNTER (__COUNTER__ - COUNTER_BASE - 1)

extern const struct gb_control_priv_data gb_control_priv_data;

static struct gb_cport *cports[CONFIG_GREYBUS_HOST_CPORT_MAX_COUNT];

struct gb_cport *gb_cport_new(const struct gb_driver *driver, const void *priv, uint8_t protocol,
			      uint16_t id)
{
	struct gb_cport *cport = gb_alloc(sizeof(struct gb_cport));
	if (!cport) {
		return NULL;
	}

	cport->driver = driver;
	cport->priv = priv;
	cport->protocol = protocol;

	return cport;
}

int gb_cport_register(struct gb_cport *cport, uint8_t id)
{
	cports[id] = cport;
	return 0;
}

const struct gb_cport *gb_cport_get(uint16_t cport)
{
	return (cport >= CONFIG_GREYBUS_HOST_CPORT_MAX_COUNT) ? NULL : cports[cport];
}

int gb_cports_init()
{
	return 0;
}

void gb_cports_deinit()
{
}
