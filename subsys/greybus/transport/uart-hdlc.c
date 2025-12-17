/*
 * Copyright (c) 2025 Ayush Singh BeagleBoard.org
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "../greybus_transport.h"
#include <greybus/greybus.h>
#include <zephyr/kernel.h>
#include <greybus-utils/manifest.h>
#include "../greybus_internal.h"
#include "hdlc.h"
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>

K_MSGQ_DEFINE(rx_msgq, sizeof(struct gb_msg_with_cport), 2, 1);

#define UART_DEVICE_NODE DT_CHOSEN(zephyr_uart_pipe)

LOG_MODULE_REGISTER(greybus_basic, CONFIG_GREYBUS_LOG_LEVEL);

struct hdlc_greybus_frame {
	uint16_t cport;
	struct gb_operation_msg_hdr hdr;
	uint8_t payload[];
} __packed;

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

static int hdlc_process_greybus_frame(const char *buffer, size_t buffer_len)
{
	struct gb_message *msg;
	int ret;
	struct hdlc_greybus_frame *gb_frame = (struct hdlc_greybus_frame *)buffer;
	size_t msg_len = buffer_len - sizeof(uint16_t);
	struct gb_operation_msg_hdr *hdr = (struct gb_operation_msg_hdr *)&buffer[sizeof(uint16_t)];

	if (sys_le16_to_cpu(gb_frame->hdr.size) > msg_len) {
		LOG_ERR("Greybus Message size is greater than received buffer.");
		return -1;
	}

	msg = gb_message_alloc(gb_hdr_payload_len(hdr), gb_frame->hdr.type,
			       gb_frame->hdr.operation_id, gb_frame->hdr.result);
	if (!msg) {
		LOG_ERR("Failed to allocate greybus message");
		return -1;
	}

	memcpy(msg->payload, gb_frame->payload, gb_message_payload_len(msg));
	// ret = ap_rx_submit(msg, sys_le16_to_cpu(gb_frame->cport));
	ret = greybus_rx_handler(sys_le16_to_cpu(gb_frame->cport), msg);
	if (ret < 0) {
		LOG_ERR("Failed to receive greybus message");
		gb_message_dealloc(msg);
		return ret;
	}

	return 0;
}

static int process_frame_callback(const void *payload, size_t payload_len, uint8_t address)
{
	switch (address) {
	case ADDRESS_GREYBUS:
		return hdlc_process_greybus_frame(payload, payload_len);
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

static int init()
{
	int ret = hdlc_init(&process_frame_callback, &send_frame_callback);
	if (ret < 0) {
		printk("Failed to initialize HDLC: %d\n", ret);
		return ret;
	}
	LOG_DBG("HDLC initialized successfully\n");

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
	return 0;
}

static int listen(uint16_t cport)
{
	LOG_DBG("Listening on cport %u\n", cport);
	return 0;
}

static int stop_listening(uint16_t cport)
{
	LOG_DBG("Stop listening on cport %u\n", cport);
	return 0;
}

static int trans_send(uint16_t cport, const struct gb_message *msg)
{
	return gb_message_hdlc_send(msg, cport);
}

const struct gb_transport_backend gb_trans_backend = {
	.init = init,
	.listen = listen,
	.stop_listening = stop_listening,
	.send = trans_send,
};
