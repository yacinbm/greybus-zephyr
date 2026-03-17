#include <zephyr/logging/log.h>
#include <greybus/greybus_messages.h>
#include "greybus_cport.h"
#include "greybus_internal.h"
#include "greybus_transport.h"

LOG_MODULE_REGISTER(greybus_host_control, CONFIG_GREYBUS_LOG_LEVEL);

#define GB_CONTROL_TYPE_CONNECTED 0x05
#define GB_CONTROL_CPORT_ID       0

int gb_control_connection_enable(const struct gb_cport *cport)
{
	struct gb_message *msg = gb_message_request_alloc(
		sizeof(struct gb_control_connected_request), GB_CONTROL_TYPE_CONNECTED, false);

	gb_transport_message_send(msg, GB_CONTROL_CPORT_ID);

	gb_message_dealloc(msg);

	return 0;
}

static void gb_control_host_probe(const struct gb_cport *cport)
{
	LOG_INF("Greybus control host probe");

	// Host control CPort starts with sending a Control Connected command
	gb_control_connection_enable(cport);
}

const struct gb_bundle_driver gb_control_bundle_driver = {
	.probe = gb_control_host_probe,
};

const struct gb_driver gb_control_driver = {};
