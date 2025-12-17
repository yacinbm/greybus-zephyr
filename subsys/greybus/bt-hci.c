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

#include <sl_btctrl_linklayer.h>
#include <sl_hci_common_transport.h>
#include <pa_conversions_efr32.h>
#include <rail.h>
#include <soc_radio.h>

static K_KERNEL_STACK_DEFINE(slz_ll_stack, CONFIG_BT_SILABS_EFR32_LINK_LAYER_STACK_SIZE);
static struct k_thread slz_ll_thread;

LOG_MODULE_REGISTER(greybus_bt_hci, CONFIG_GREYBUS_LOG_LEVEL);

/* Reserved buffer for rx data. */
#define MAX_RX_BUF_SIZE 64

#define GB_BT_HCI_TRANSFER 0x02

static uint16_t cport_index;

void BTLE_LL_EventRaise(uint32_t events);
void BTLE_LL_Process(uint32_t events);
int16_t BTLE_LL_SetMaxPower(int16_t power);

/* Events mask for Link Layer */
static atomic_t sli_btctrl_events;

/* Semaphore for Link Layer */
K_SEM_DEFINE(slz_ll_sem, 0, 1);

/* Store event flags and increment the LL semaphore */
void BTLE_LL_EventRaise(uint32_t events)
{
	atomic_or(&sli_btctrl_events, events);
	k_sem_give(&slz_ll_sem);
}

/**
 * The HCI driver thread simply waits for the LL semaphore to signal that
 * it has an event to handle, whether it's from the radio, its own scheduler,
 * or an HCI event to pass upstairs. The BTLE_LL_Process function call will
 * take care of all of them, and add HCI events to the HCI queue when applicable.
 */
static void slz_ll_thread_func(void *p1, void *p2, void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	while (true) {
		uint32_t events;

		k_sem_take(&slz_ll_sem, K_FOREVER);
		events = atomic_clear(&sli_btctrl_events);
		BTLE_LL_Process(events);
	}
}

bool sli_pending_btctrl_events(void)
{
	return false; /* TODO: check if this should really return false! */
}

void sli_btctrl_events_init(void)
{
	atomic_clear(&sli_btctrl_events);
}

static void slz_set_tx_power(int16_t max_power_dbm)
{
	const int16_t max_power_cbm = max_power_dbm * 10;
	const int16_t actual_max_power_cbm = BTLE_LL_SetMaxPower(max_power_cbm);
	const int16_t actual_max_power_dbm = DIV_ROUND_CLOSEST(actual_max_power_cbm, 10);

	if (actual_max_power_dbm != max_power_dbm) {
		LOG_WRN("Unable to set max TX power to %d dBm, actual max is %d dBm", max_power_dbm,
			actual_max_power_dbm);
	}
}

/**
 * @brief Transmit HCI message using the currently used transport layer.
 * The HCI calls this function to transmit a full HCI message.
 * @param[in] data Packet type followed by HCI packet data.
 * @param[in] len Length of the `data` parameter
 * @return 0 - on success, or non-zero on failure.
 */
uint32_t hci_common_transport_transmit(uint8_t *data, int16_t len)
{
	struct gb_message *req;
	int ret;

	req = gb_message_request_alloc(len, GB_BT_HCI_TRANSFER, true);
	if (!req) {
		LOG_ERR("Failed to allocate message");
		return -ENOMEM;
	}

	LOG_DBG("Transmitting HCI message of length %d", len);

	memcpy(req->payload, data, len);

	ret = gb_transport_message_send(req, cport_index);
	if (ret != 0) {
		LOG_ERR("gb_transport_message_send failed, status=%d", ret);
	}
	gb_message_dealloc(req);

	sl_btctrl_hci_transmit_complete(0);

	return 0;
}

static void gb_bt_hci_handler(const void *priv, struct gb_message *msg, uint16_t cport)
{
	int ret;

	switch (gb_message_type(msg)) {
	case GB_BT_HCI_TRANSFER:
		ret = sl_btctrl_hci_receive(msg->payload, gb_message_payload_len(msg), true);
		if (ret != 0) {
			LOG_ERR("sl_btctrl_hci_receive failed, status=%d", ret);
		}
		break;
	default:
		LOG_ERR("Invalid type %d", gb_message_type(msg));
		return gb_transport_message_empty_response_send(msg, GB_OP_PROTOCOL_BAD, cport);
	}
}

static void gb_bt_hci_connected(const void *priv, uint16_t cport)
{
	// struct hci_data *hci = dev->data;
	sl_status_t sl_status;
	int ret;

	cport_index = cport;

	sli_btctrl_events_init();

	k_thread_create(&slz_ll_thread, slz_ll_stack, K_KERNEL_STACK_SIZEOF(slz_ll_stack),
			slz_ll_thread_func, NULL, NULL, NULL,
			K_PRIO_COOP(CONFIG_BT_SILABS_EFR32_LL_THREAD_PRIO), 0, K_NO_WAIT);
	k_thread_name_set(&slz_ll_thread, "EFR32 LL");

	LOG_DBG("BT HCI Connected");
	sl_rail_util_pa_init();

	/* Initialize Controller features based on Kconfig values */
	sl_status = sl_btctrl_init();
	if (sl_status != SL_STATUS_OK) {
		LOG_ERR("sl_bt_controller_init failed, status=%d", sl_status);
		ret = -EIO;
		goto deinit;
	}

	slz_set_tx_power(CONFIG_BT_CTLR_TX_PWR_ANTENNA);

	if (IS_ENABLED(CONFIG_PM)) {
		RAIL_ConfigSleep(sli_btctrl_get_radio_context_handle(),
				 RAIL_SLEEP_CONFIG_TIMERSYNC_ENABLED);
		RAIL_Status_t status = RAIL_InitPowerManager();

		if (status != RAIL_STATUS_NO_ERROR) {
			LOG_ERR("RAIL: failed to initialize power management, status=%d", status);
			ret = -EIO;
			goto deinit;
		}
	}

	/* Set up interrupts after Controller init, because it will overwrite them. */
	rail_isr_installer();

	LOG_DBG("SiLabs BT HCI started");

	return;
deinit:
	sl_btctrl_deinit(); /* No-op if controller initialization failed */
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
