/*
 * Copyright (c) 2020 Friedt Professional Engineering Services, Inc
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <zephyr/init.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(greybus_host, CONFIG_GREYBUS_LOG_LEVEL);

int greybus_host_init(void)
{
	LOG_INF("Greybus host is active");
	return 0;
}

SYS_INIT(greybus_host_init, APPLICATION, CONFIG_GREYBUS_HOST_INIT_PRIORITY);
