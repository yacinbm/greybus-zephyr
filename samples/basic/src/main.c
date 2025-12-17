/*
 * Copyright (c) 2025 Ayush Singh BeagleBoard.org
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include <greybus/greybus.h>

#ifdef CONFIG_PRINT_MANIFEST
#include "greybus-utils/manifest.h"
#endif // CONFIG_PRINT_MANIFEST

LOG_MODULE_REGISTER(main, CONFIG_GREYBUS_LOG_LEVEL);

// #define UART_DEVICE_NODE DT_CHOSEN(zephyr_bt_mon_uart)
// static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

int main(void)
{
#ifdef CONFIG_PRINT_MANIFEST
	static char manifest[1024];
	int ret = manifest_create(manifest, ARRAY_SIZE(manifest));
	LOG_DBG("Manifest Size: %d\n", ret);

	manifest_print(manifest);
#endif // CONFIG_PRINT_MANIFEST

	// if (!device_is_ready(uart_dev)) {
	// 	LOG_ERR("UART device not ready");
	// 	return -ENODEV;
	// }

	k_sleep(K_FOREVER);

	return 0;
}
