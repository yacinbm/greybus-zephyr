#include <zephyr/logging/log.h>
#include "greybus_cport.h"
#include "greybus-manifest.h"
#include "greybus_heap.h"
#include <greybus/host.h>

LOG_MODULE_REGISTER(greybus_cport_host, CONFIG_GREYBUS_LOG_LEVEL);

extern const struct gb_driver gb_control_host_driver;

/* Reset the counter to 0 */
enum {
	COUNTER_BASE = __COUNTER__
};
#define LOCAL_COUNTER (__COUNTER__ - COUNTER_BASE - 1)

extern const struct gb_control_priv_data gb_control_priv_data;

static struct gb_cport *cports[CONFIG_GREYBUS_HOST_CPORT_MAX_COUNT];

static int cport_add(struct gb_host *host, struct gb_cport *cport, uint8_t id)
{
	host->cports[id] = cport;
	return 0;
}

struct gb_cport *gb_cport_add(struct gb_host *host, const struct gb_bundle_driver *bundle_driver,
			      const struct gb_driver *driver, void *priv, uint8_t protocol,
			      uint8_t id)
{
	struct gb_cport *cport = gb_alloc(sizeof(struct gb_cport));
	if (!cport) {
		return NULL;
	}

	cport->driver = driver;
	cport->priv = priv;
	cport->protocol = protocol;

	cport_add(host, cport, id);

	return cport;
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
