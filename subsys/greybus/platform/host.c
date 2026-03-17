/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <greybus/greybus.h>
#include <greybus/host.h>
#include "../greybus_cport.h"
#include "../greybus_internal.h"
#include "../greybus_transport.h"
#include "../greybus_cport.h"
#include "../greybus-manifest.h"

extern const struct gb_driver gb_control_driver;
extern const struct gb_bundle_driver gb_control_bundle_driver;

struct gb_host {
	struct gb_cport *cports;
	uint8_t cport_count;
};

LOG_MODULE_REGISTER(greybus_host, CONFIG_GREYBUS_LOG_LEVEL);

static const struct greybush_bundle_class_match *greybush_cport_match(struct gb_cport *cport)
{
	STRUCT_SECTION_FOREACH(greybush_class_node, c_node) {
		const struct greybush_bundle_class_match *m = c_node->filter;

		if (m->class == cport->bundle && m->protocol == cport->protocol) {
			return m;
		}
	}

	return NULL;
}

int greybus_host_init(void)
{
	const struct gb_transport_backend *xport = gb_transport_get_backend();

	LOG_DBG("Greybus initializing..");

	int r = gb_init(xport);
	if (r < 0) {
		LOG_ERR("gb_init() failed: %d", r);
		return r;
	}

	// Register Control CPort
	struct gb_cport *control_cport =
		gb_cport_new(&gb_control_driver, NULL, GREYBUS_PROTOCOL_CONTROL, 0);
	if (!control_cport) {
		LOG_ERR("Failed to create control cport");
		return -ENOMEM;
	}

	r = gb_cport_register(control_cport, 0);
	if (r < 0) {
		LOG_ERR("Failed to register control cport");
		return r;
	}

	gb_control_bundle_driver.probe(control_cport);

	LOG_INF("Greybus host is active");
	return 0;
}

SYS_INIT(greybus_host_init, APPLICATION, CONFIG_GREYBUS_HOST_INIT_PRIORITY);
