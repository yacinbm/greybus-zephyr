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
#include <sl_btctrl_linklayer.h>

LOG_MODULE_REGISTER(greybus_bt_hci, CONFIG_GREYBUS_LOG_LEVEL);

/* Reserved buffer for rx data. */
#define MAX_RX_BUF_SIZE 64

static void gb_bt_hci_handler(const void *priv, struct gb_message *msg, uint16_t cport)
{
	switch (gb_message_type(msg)) {
	default:
		LOG_ERR("Invalid type");
		return gb_transport_message_empty_response_send(msg, GB_OP_PROTOCOL_BAD, cport);
	}
}

static void gb_bt_hci_connected(const void *priv, uint16_t cport)
{
	LOG_DBG("BT HCI Connected");

	int ret = sl_btctrl_hci_receive(buf->data, buf->len, true);
	if (!ret) {
		return;
	}
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
