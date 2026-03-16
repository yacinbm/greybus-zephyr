/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <zephyr/init.h>
#include <zephyr/logging/log.h>
#include <greybus/greybus.h>
#include <greybus/host.h>
#include "../greybus_cport.h"
#include "../greybus_internal.h"
#include "../greybus_transport.h"

LOG_MODULE_REGISTER(greybus_host, CONFIG_GREYBUS_LOG_LEVEL);

static const struct greybush_bundle_class_match *greybush_cport_match(struct gb_cport *cport)
{
	STRUCT_SECTION_FOREACH(greybush_class_node, c_node) {
		const struct greybush_bundle_class_match *m = c_node->filter;

		if (m->class == cport->bundle && m->protocol == cport->protocol)
			return m;
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

	LOG_INF("Greybus host is active");
	return 0;
}

SYS_INIT(greybus_host_init, APPLICATION, CONFIG_GREYBUS_HOST_INIT_PRIORITY);
