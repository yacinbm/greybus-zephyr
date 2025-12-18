/*
 * Copyright (c) 2015 Silicon Laboratories, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 * contributors may be used to endorse or promote products derived from this
 * software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "greybus_transport.h"
#include <zephyr/logging/log.h>
#include <greybus/greybus_protocols.h>
#include "greybus_internal.h"
#include <zephyr/drivers/bluetooth.h>
#include <zephyr/bluetooth/hci_raw.h>

LOG_MODULE_REGISTER(greybus_bt_hci, CONFIG_GREYBUS_LOG_LEVEL);

/* Reserved buffer for rx data. */
#define MAX_RX_BUF_SIZE    64
#define PACKET_TYPE        0
#define HCI_CMD            0x01
#define HCI_ACL            0x02
#define GB_BT_HCI_TRANSFER 0x02

static uint16_t cport_index;

static struct k_fifo rx_queue;
static K_THREAD_STACK_DEFINE(bt_rx_thread_stack, CONFIG_BT_HCI_TX_STACK_SIZE);
static struct k_thread bt_rx_thread_data;

static void gb_bt_hci_handler(const void *priv, struct gb_message *msg, uint16_t cport)
{
	int ret;

	union {
		struct bt_hci_cmd_hdr *cmd_hdr;
		struct bt_hci_acl_hdr *acl_hdr;
	} hci_hdr;

	switch (gb_message_type(msg)) {
	case GB_BT_HCI_TRANSFER: {
		struct net_buf *buf = NULL;
		hci_hdr.cmd_hdr = (struct bt_hci_cmd_hdr *)&msg->payload[1];

		switch (msg->payload[PACKET_TYPE]) {
		case HCI_CMD:
			buf = bt_buf_get_tx(BT_BUF_CMD, K_NO_WAIT, hci_hdr.cmd_hdr,
					    sizeof(*hci_hdr.cmd_hdr));
			if (!buf) {
				LOG_ERR("No available command buffers!");
				break;
			}

			net_buf_add_mem(buf, &msg->payload[4], hci_hdr.cmd_hdr->param_len);
			break;

		case HCI_ACL:
			buf = bt_buf_get_tx(BT_BUF_ACL_OUT, K_NO_WAIT, hci_hdr.acl_hdr,
					    sizeof(*hci_hdr.acl_hdr));
			if (!buf) {
				LOG_ERR("No available ACL buffers!");
				break;
			}

			net_buf_add_mem(buf, &msg->payload[5],
					sys_le16_to_cpu(hci_hdr.acl_hdr->len));
			break;
		default:
			LOG_ERR("Unknown BT HCI buf type");
			break;
		}

		LOG_DBG("buf %p type %u len %u", buf, buf->data[0], buf->len);

		ret = bt_send(buf);
		if (ret) {
			LOG_ERR("Unable to send (ret %d)", ret);
			net_buf_unref(buf);
		}

		LOG_DBG("Sent %d bytes to Bluetooth", buf->len);
		break;
	}
	default:
		LOG_ERR("Invalid type %d", gb_message_type(msg));
		return gb_transport_message_empty_response_send(msg, GB_OP_PROTOCOL_BAD, cport);
	}
}

static void bt_rx_thread(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (1) {
		struct net_buf *buf = k_fifo_get(&rx_queue, K_FOREVER);
		if (buf) {
			struct gb_message *msg = gb_message_request_alloc_with_payload(
				buf->data, buf->len, GB_BT_HCI_TRANSFER, true);
			gb_transport_message_send(msg, cport_index);

			LOG_DBG("Sent %d bytes to Greybus", buf->len);
			net_buf_unref(buf);
		}
	}
}

static void gb_bt_hci_connected(const void *priv, uint16_t cport)
{
	// struct hci_data *hci = dev->data;
	k_tid_t rx_id;
	int err;

	k_fifo_init(&rx_queue);

	cport_index = cport;

	err = bt_enable_raw(&rx_queue);
	if (err) {
		LOG_ERR("bt_enable_raw: %d; aborting", err);
		return;
	}

	/* Spawn the RX thread, which sends data received from the radio to the Greybus host
	 */
	rx_id = k_thread_create(&bt_rx_thread_data, bt_rx_thread_stack,
				K_THREAD_STACK_SIZEOF(bt_rx_thread_stack), bt_rx_thread, NULL, NULL,
				NULL, K_PRIO_COOP(7), 0, K_NO_WAIT);
	k_thread_name_set(&bt_rx_thread_data, "bt_rx_thread");

	LOG_DBG("SiLabs BT HCI started");

	return;
}

static void gb_bt_hci_disconnected(const void *priv)
{
	LOG_DBG("BT HCI Disconnected");
}

const struct gb_driver gb_bt_hci_driver = {
	.op_handler = gb_bt_hci_handler,
	.connected = gb_bt_hci_connected,
	.disconnected = gb_bt_hci_disconnected,
};
