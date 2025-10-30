/*
 * Copyright (c) 2025 Ayush Singh, BeagleBoard.org
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "greybus/greybus_messages.h"
#include <zephyr/ztest.h>
#include <greybus/greybus.h>
#include <greybus-utils/manifest.h>
#include <greybus/greybus_log.h>
#include <greybus/greybus_raw.h>

#define BUF_SIZE 128

struct gb_msg_with_cport gb_transport_get_message(void);

static uint8_t greybus_raw_cb(uint32_t len, const uint8_t *data, void *priv)
{
	memcpy(priv, data, len);

	return GB_OP_SUCCESS;
}

ZTEST_SUITE(greybus_raw_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(greybus_raw_tests, test_cport_count)
{
	zassert_equal(GREYBUS_CPORT_COUNT, 3, "Invalid number of cports");
	zassert_equal(GREYBUS_RAW_CPORT_START, 1, "Raw cports should start from 1 here");
}

ZTEST(greybus_raw_tests, test_send_data)
{
	int i, ret;
	uint8_t cb_buf[BUF_SIZE];
	struct gb_msg_with_cport resp;
	struct gb_raw_send_request *req_data;
	struct gb_message *req =
		gb_message_request_alloc(sizeof(*req_data) + BUF_SIZE, GB_RAW_TYPE_SEND, false);

	req_data = (struct gb_raw_send_request *)req->payload;
	req_data->len = sys_cpu_to_le32(BUF_SIZE);
	for (i = 0; i < BUF_SIZE; i++) {
		req_data->data[i] = i;
	}

	ret = greybus_raw_register(greybus_raw_cb, cb_buf);
	zassert_equal(ret, 0, "Expect the first registered raw id 0");

	greybus_rx_handler(1, req);
	resp = gb_transport_get_message();
	zassert_equal(resp.cport, 1, "Invalid cport");
	zassert(gb_message_is_success(resp.msg), "Request failed");
	zassert_equal(gb_message_type(resp.msg), GB_RESPONSE(GB_RAW_TYPE_SEND),
		      "Invalid response type");
	zassert_equal(gb_message_payload_len(resp.msg), 0, "Invalid response size");
	for (i = 0; i < BUF_SIZE; i++) {
		zassert_equal(cb_buf[i], i, "Invalid data");
	}

	gb_message_dealloc(resp.msg);

	ret = greybus_raw_send_data(0, BUF_SIZE, cb_buf);
	zassert_equal(ret, 0, "Failed to send data");
	resp = gb_transport_get_message();
	zassert_equal(resp.cport, 1, "Invalid cport");
	zassert_equal(gb_message_type(resp.msg), GB_RAW_TYPE_SEND, "Invalid response type");
	zassert_equal(gb_message_payload_len(resp.msg), sizeof(*req_data) + BUF_SIZE,
		      "Invalid response size");

	req_data = (struct gb_raw_send_request *)resp.msg->payload;
	zassert_equal(sys_le32_to_cpu(req_data->len), BUF_SIZE, "Invalid data length");
	for (i = 0; i < BUF_SIZE; i++) {
		zassert_equal(req_data->data[i], i, "Invalid data");
	}

	req = gb_message_response_alloc_from_req(NULL, 0, resp.msg, GB_OP_SUCCESS);
	greybus_rx_handler(1, req);

	gb_message_dealloc(resp.msg);
	gb_message_dealloc(req);
}
