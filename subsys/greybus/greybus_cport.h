/*
 * Copyright (c) 2025 Ayush Singh BeagleBoard.org
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _GREYBUS_CPORT_H_
#define _GREYBUS_CPORT_H_

#include <greybus/greybus.h>

struct gb_host;
struct gb_cport {
	const struct gb_driver *driver;
	const void *priv;
	uint8_t bundle;
	uint8_t protocol;
};

#define GB_CPORT(_priv, _bundle, _protocol, _driver)                                               \
	{                                                                                          \
		.bundle = _bundle,                                                                 \
		.protocol = _protocol,                                                             \
		.priv = _priv,                                                                     \
		.driver = _driver,                                                                 \
	}

struct gb_cport *gb_cport_add(struct gb_host *host, const struct gb_driver *driver,
			      const void *priv, uint8_t protocol, uint8_t id);
const struct gb_cport *gb_cport_get(uint16_t cport);

/**
 * Initialize all cports.
 */
int gb_cports_init();

void gb_cports_deinit();

#endif // _GREYBUS_CPORT_H_
