/*
 * Copyright (c) 2025 Ayush Singh BeagleBoard.org
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/printk.h>

#include "hdlc.h"

#ifdef CONFIG_PRINT_MANIFEST
#include "greybus-utils/manifest.h"
#endif // CONFIG_PRINT_MANIFEST

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_uart_pipe)

LOG_MODULE_REGISTER(greybus_basic, CONFIG_GREYBUS_LOG_LEVEL);

static const struct device *const uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

static void serial_callback(const struct device *dev, void *user_data)
{
	ARG_UNUSED(user_data);

	uint8_t *buf;
	int ret;

	if (!uart_irq_update(dev) && !uart_irq_rx_ready(dev)) {
		return;
	}

	ret = hdlc_rx_start(&buf);
	if (ret == 0) {
		/* No space */
		LOG_ERR("No more space for HDLC receive");
		return;
	}

	ret = uart_fifo_read(dev, buf, ret);
	if (ret < 0) {
		/* Something went wrong */
		LOG_ERR("Failed to read UART");
		return;
	}

	ret = hdlc_rx_finish(ret);
	if (ret < 0) {
		/* Some error */
		LOG_ERR("Filed to write data to hdlc buffer");
		return;
	}
}

static int process_frame_callback(const void *payload, size_t payload_len, uint8_t address)
{
	switch (address) {
	case ADDRESS_GREYBUS:
		LOG_ERR("Ignore Greybus Frame");
		break;
	case ADDRESS_CONTROL:
		LOG_ERR("Ignore Control Frame");
		break;
	case ADDRESS_DBG:
		LOG_WRN("Ignore DBG Frame");
		break;
	}
	return 0;
}

static int send_frame_callback(const uint8_t *buffer, size_t buffer_len)
{
	size_t i;

	for (i = 0; i < buffer_len; ++i) {
		uart_poll_out(uart_dev, buffer[i]);
	}

	return i;
}

int main(void)
{
#ifdef CONFIG_PRINT_MANIFEST
	static char manifest[1024];
#endif
	int ret;

	ret = hdlc_init(&process_frame_callback, &send_frame_callback);
	if (ret < 0) {
		printk("Failed to initialize HDLC: %d\n", ret);
		return ret;
	}
	LOG_DBG("HDLC initialized successfully\n");

#ifdef CONFIG_PRINT_MANIFEST
	ret = manifest_create(manifest, ARRAY_SIZE(manifest));
	LOG_DBG("Manifest Size: %d\n", ret);

	manifest_print(manifest);
#endif // CONFIG_PRINT_MANIFEST

	ret = uart_irq_callback_user_data_set(uart_dev, serial_callback, NULL);
	if (ret < 0) {
		if (ret == -ENOTSUP) {
			LOG_ERR("Interrupt-driven UART API support not enabled\n");
		} else if (ret == -ENOSYS) {
			LOG_ERR("UART device does not support interrupt-driven API\n");
		} else {
			LOG_ERR("Error setting UART callback: %d\n", ret);
		}
		return ret;
	}
	LOG_DBG("UART callback set successfully\n");

	uart_irq_rx_enable(uart_dev);

	k_sleep(K_FOREVER);

	return 0;
}
