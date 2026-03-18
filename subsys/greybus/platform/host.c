/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <stdbool.h>
#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <greybus/greybus.h>
#include <greybus/host.h>
#include "../greybus_cport.h"
#include "../greybus_internal.h"
#include "../greybus_transport.h"
#include "../greybus_cport.h"
#include "../greybus-manifest.h"
#include "../greybus_heap.h"

extern const struct gb_driver gb_control_driver;
extern const struct gb_bundle_driver gb_control_bundle_driver;
int gb_control_get_manifest(void);

LOG_MODULE_REGISTER(greybus_host, CONFIG_GREYBUS_LOG_LEVEL);

const struct greybush_class_node *greybush_cport_match(uint8_t class, uint8_t protocol)
{
	STRUCT_SECTION_FOREACH(greybush_class_node, c_node) {
		const struct greybush_class_node *node = c_node;
		const struct greybush_bundle_class_match *filter = node->filter;

		if (filter->class == class && filter->protocol == protocol) {
			return node;
		}
	}

	return NULL;
}

static int gb_manifest_parse()
{
	int r;

	r = gb_control_get_manifest();
	if (r < 0) {
		LOG_ERR("Failed to get manifest: %d", r);
		return r;
	}

	return 0;
}

static int greybus_host_init(void)
{
	const struct gb_transport_backend *xport = gb_transport_get_backend();

	LOG_DBG("Greybus initializing..");

	int r = gb_init(xport);
	if (r < 0) {
		LOG_ERR("gb_init() failed: %d", r);
		return r;
	}

	// Register Control CPort
	struct gb_cport *control_cport = gb_cport_add(&gb_control_bundle_driver, &gb_control_driver,
						      NULL, GREYBUS_PROTOCOL_CONTROL, 0);
	if (!control_cport) {
		LOG_ERR("Failed to create control cport");
		return -ENOMEM;
	}

	gb_control_bundle_driver.probe(control_cport);

	gb_manifest_parse();

	LOG_INF("Greybus host is active");
	return 0;
}

SYS_INIT(greybus_host_init, APPLICATION, CONFIG_GREYBUS_HOST_INIT_PRIORITY);
