#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <greybus/greybus_messages.h>
#include <greybus/host.h>
#include "greybus_cport.h"
#include "greybus_internal.h"
#include "greybus_transport.h"
#include "greybus-manifest.h"

LOG_MODULE_REGISTER(greybus_host_control, CONFIG_GREYBUS_LOG_LEVEL);

#define GB_CONTROL_TYPE_CONNECTED 0x05
#define GB_CONTROL_CPORT_ID       0

#define GB_HOST_CTRL_THREAD_STACK_SIZE 1024
#define GB_HOST_CTRL_THREAD_PRIORITY   10

K_MSGQ_DEFINE(gb_host_ctrl_msgq, sizeof(struct gb_message *), 10, 1);

const struct greybush_class_node *greybush_cport_match(uint8_t class, uint8_t protocol);
static int gb_control_get_manifest_response(struct gb_message *resp);

static void gb_control_host_op_handler(const void *priv, struct gb_message *msg, uint16_t cport)
{
	(void)priv;
	(void)cport;

	k_msgq_put(&gb_host_ctrl_msgq, &msg, K_FOREVER);
}

static void gb_host_ctrl_thread_entry(void *p1, void *p2, void *p3)
{
	struct gb_message *msg;

	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	for (;;) {
		k_msgq_get(&gb_host_ctrl_msgq, &msg, K_FOREVER);

		switch (gb_message_type(msg)) {
			case GB_CONTROL_TYPE_GET_MANIFEST:
				gb_control_get_manifest_response(msg);
				break;
			default:
				break;
		}

		gb_message_dealloc(msg);
	}
}

K_THREAD_STACK_DEFINE(gb_host_ctrl_thread_stack, GB_HOST_CTRL_THREAD_STACK_SIZE);
static struct k_thread gb_host_ctrl_thread_data;

static int gb_host_ctrl_thread_init(void)
{
	k_tid_t tid = k_thread_create(&gb_host_ctrl_thread_data,
				      gb_host_ctrl_thread_stack,
				      K_THREAD_STACK_SIZEOF(gb_host_ctrl_thread_stack),
				      gb_host_ctrl_thread_entry,
				      NULL, NULL, NULL,
				      GB_HOST_CTRL_THREAD_PRIORITY, 0, K_NO_WAIT);
	return tid ? 0 : -1;
}

int gb_control_get_manifest(void)
{
	struct gb_message *req;

	req = gb_message_request_alloc(0, GB_CONTROL_TYPE_GET_MANIFEST, false);
	if (!req) {
		LOG_ERR("Failed to allocate message");
		return -ENOMEM;
	}

	req->header.size = sizeof(struct gb_message);

	gb_transport_message_send(req, 0);
	gb_message_dealloc(req);

	return 0;
}

static int gb_control_get_manifest_response(struct gb_message *resp)
{
	const struct greybush_class_node *node;
	struct greybush_class_data *data;
	struct greybus_manifest *manifest;
	struct gb_cport *cport;
	size_t desc_remaining;
	const uint8_t *desc_ptr;
	uint8_t found_bundle_id = 0;
	uint8_t found_class = 0;
	bool have_bundle = false;
	uint8_t cport_id, protocol_id, class_id;
	int retval;

	__ASSERT(gb_message_is_response(resp), "GB Message should be a response");
	__ASSERT(gb_message_is_success(resp), "GB Message should be a success");
	__ASSERT(resp->header.size >= sizeof(struct gb_message) + sizeof(struct greybus_manifest),
		 "GB Control Get Manifest response is too short");

	manifest = (struct greybus_manifest *)resp->payload;
	desc_remaining = sys_le16_to_cpu(manifest->header.size);
	if (desc_remaining <= sizeof(struct greybus_manifest_header)) {
		retval = -EINVAL;
		goto exit;
	}

	desc_remaining -= sizeof(struct greybus_manifest_header);
	desc_ptr = (const uint8_t *)&manifest->descriptors[0];

	/* Walk descriptors: find first bundle, then first cport belonging to that bundle. */
	while (desc_remaining >= sizeof(struct greybus_descriptor_header)) {
		struct greybus_descriptor *desc = (struct greybus_descriptor *)desc_ptr;
		struct greybus_descriptor_header *hdr = &desc->header;

		uint16_t desc_size = sys_le16_to_cpu(hdr->size);

		switch (hdr->type) {
		case GREYBUS_TYPE_BUNDLE: {
			const struct greybus_descriptor_bundle *bundle = &desc->bundle;

			/* Skip control bundle (already registered); use first non-control bundle. */
			if (!have_bundle && bundle->class != GREYBUS_CLASS_CONTROL) {
				found_bundle_id = bundle->id;
				found_class = bundle->class;
				have_bundle = true;
			}
			break;
		}
		case GREYBUS_TYPE_CPORT: {
			const struct greybus_descriptor_cport *cp = &desc->cport;

			if (have_bundle && cp->bundle == found_bundle_id) {
				class_id = found_class;
				protocol_id = cp->protocol_id;
				cport_id = sys_le16_to_cpu(cp->id);

				node = greybush_cport_match(class_id, protocol_id);
				if (!node) {
					LOG_ERR("No matching class %u and protocol_id %u found", class_id, protocol_id);
					return -ENODEV;
				}

				data = node->c_data;
				cport = gb_cport_add(data->api, data->driver, data->priv,
						     protocol_id, (uint8_t)cport_id);
				if (!cport) {
					LOG_ERR("Failed to create cport");
					return -ENOMEM;
				}

				data->api->probe(cport);

				retval = 0;
				goto exit;
			}
			break;
		}
		default:
			/* Ignore INTERFACE, STRING, and unknown descriptors */
			break;
		}

		desc_ptr += desc_size;
		desc_remaining -= desc_size;
	}

	retval = -ENOENT;

exit:
	return retval;
}

int gb_control_connection_enable(const struct gb_cport *cport)
{
	struct gb_message *msg = gb_message_request_alloc(
		sizeof(struct gb_control_connected_request), GB_CONTROL_TYPE_CONNECTED, false);

	struct gb_control_connected_request *req =
		(struct gb_control_connected_request *)msg->payload;
	req->cport_id = sys_cpu_to_le16(cport->id);

	gb_transport_message_send(msg, GB_CONTROL_CPORT_ID);

	gb_message_dealloc(msg);

	return 0;
}

static int gb_control_host_probe(const struct gb_cport *cport)
{
	int r;

	LOG_INF("Greybus control host probe");

	r = gb_host_ctrl_thread_init();
	if (r < 0) {
		LOG_ERR("Failed to start control host thread");
		return r;
	}

	/* Host control CPort starts with sending a Control Connected command */
	return gb_control_connection_enable(cport);
}

const struct gb_bundle_driver gb_control_bundle_driver = {
	.probe = gb_control_host_probe,
};

const struct gb_driver gb_control_driver = {
	.op_handler = gb_control_host_op_handler,
};
