/*
 * Copyright (c) 2017, Christian Taedcke
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT silabs_greybus_gpio_port

#include <errno.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/gpio/gpio_utils.h>
#include <zephyr/irq.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/util.h>
#include <zephyr/sys/util_macro.h>

#include <greybus/greybus_messages.h>
#include <greybus/host.h>

#include "../greybus_internal.h"
#include "../greybus_transport.h"


LOG_MODULE_REGISTER(greybush_gpio, CONFIG_GREYBUS_LOG_LEVEL);

struct gpio_greybush_data {
	uint16_t cport;
	uint32_t activated;
	u8 ngpios;
};

static int gpio_greybus_activate(const struct device *dev, gpio_pin_t pin)
{
	return 0;
}


static int gpio_greybus_deactivate(const struct device *dev, gpio_pin_t pin)
{
	return 0;
}

static int gpio_greybush_configure(const struct device *dev,
				   gpio_pin_t pin,
				   gpio_flags_t flags)
{
	struct gpio_greybush_data *data = dev->data;
	__u8 gpio_index = (__u8)pin;
	unsigned int out = 0U;
	struct gb_message *req;
	int ret;

	if (pin >= data->ngpios)
		return -EINVAL;

	if (!IS_BIT_SET(data->activated, pin))
		if (!(flags & (GPIO_OUTPUT | GPIO_INPUT))) {
			return 0;

		ret = gpio_greybus_activate(dev, pin);
		if (ret)
			return ret;

		data->activated |= BIT(pin);
	}

	if (flags & GPIO_OUTPUT) {
		struct gb_gpio_direction_out_request *req_data;

		req = gb_message_request_alloc(sizeof(*req_data), GB_GPIO_TYPE_DIRECTION_OUT, false);
		if (!req) {
			LOG_ERR("Failed to allocate message");
			return -ENOMEM;
		}

		req_data = (struct gb_gpio_direction_out_request *)req->payload;

		/* Following modes enable both output and input */
		if (flags & GPIO_OUTPUT_INIT_HIGH) {
			out = 1U;
		} else if (flags & GPIO_OUTPUT_INIT_LOW) {
			out = 0U;
		} else {
			__ASSERT(dev, "todo!()");
		}

		req_data->which = gpio_index;
		req_data->value = out;
		req->header.size = sizeof(struct gb_message) + sizeof(struct gb_gpio_direction_out_request);

	} else if (flags & GPIO_INPUT) {
		struct gb_gpio_direction_in_request *req_data;

		req = gb_message_request_alloc(sizeof(*req_data), GB_GPIO_TYPE_DIRECTION_IN, false);
		if (!req) {
			LOG_ERR("Failed to allocate message");
			return -ENOMEM;
		}

		req_data = (struct gb_gpio_direction_in_request *)req->payload;

		req_data->which = gpio_index;
		req->header.size = sizeof(struct gb_message) + sizeof(struct gb_gpio_direction_in_request);
	} else {
		gpio_greybus_deactivate(dev, gpio_index);
		return 0;
	}

	gb_transport_message_send(req, data->cport);
	gb_message_dealloc(req);

	return 0;
}

#ifdef CONFIG_GPIO_GET_CONFIG
static int gpio_gecko_get_config(const struct device *dev,
				 gpio_pin_t pin,
				 gpio_flags_t *out_flags)
{
	const struct gpio_gecko_config *config = dev->config;
	GPIO_Port_TypeDef gpio_index = config->gpio_index;
	GPIO_Mode_TypeDef mode;
	unsigned int out;
	gpio_flags_t flags = 0;

	mode = GPIO_PinModeGet(gpio_index, pin);
	out = GPIO_PinOutGet(gpio_index, pin);

	switch (mode) {
	case gpioModeWiredAnd:
		flags = GPIO_OUTPUT | GPIO_OPEN_DRAIN;

		if (out) {
			flags |= GPIO_OUTPUT_HIGH;
		} else {
			flags |= GPIO_OUTPUT_LOW;
		}

		break;
	case gpioModeWiredOr:
		flags = GPIO_OUTPUT | GPIO_OPEN_SOURCE;

		if (out) {
			flags |= GPIO_OUTPUT_HIGH;
		} else {
			flags |= GPIO_OUTPUT_LOW;
		}

		break;
	case gpioModePushPull:
		flags = GPIO_OUTPUT | GPIO_PUSH_PULL;

		if (out) {
			flags |= GPIO_OUTPUT_HIGH;
		} else {
			flags |= GPIO_OUTPUT_LOW;
		}

		break;
	case gpioModeInputPull:
		flags = GPIO_INPUT;

		if (out) {
			flags |= GPIO_PULL_UP;
		} else {
			flags |= GPIO_PULL_DOWN;
		}

		break;
	case gpioModeInput:
		flags = GPIO_INPUT;
		break;
	case gpioModeDisabled:
		flags = GPIO_DISCONNECTED;
		break;
	default:
		break;
	}

	*out_flags = flags;

	return 0;
}
#endif

#if 0
static int gpio_gecko_port_get_raw(const struct device *dev, uint32_t *value)
{
	const struct gpio_gecko_config *config = dev->config;
	GPIO_Port_TypeDef gpio_index = config->gpio_index;

	*value = GPIO_PortInGet(gpio_index);

	return 0;
}

static int gpio_gecko_port_set_masked_raw(const struct device *dev,
					  uint32_t mask,
					  uint32_t value)
{
	const struct gpio_gecko_config *config = dev->config;
	GPIO_Port_TypeDef gpio_index = config->gpio_index;

	GPIO_PortOutSetVal(gpio_index, value, mask);

	return 0;
}

static int gpio_gecko_port_set_bits_raw(const struct device *dev,
					uint32_t mask)
{
	const struct gpio_gecko_config *config = dev->config;
	GPIO_Port_TypeDef gpio_index = config->gpio_index;

	GPIO_PortOutSet(gpio_index, mask);

	return 0;
}

static int gpio_gecko_port_clear_bits_raw(const struct device *dev,
					  uint32_t mask)
{
	const struct gpio_gecko_config *config = dev->config;
	GPIO_Port_TypeDef gpio_index = config->gpio_index;

	GPIO_PortOutClear(gpio_index, mask);

	return 0;
}

static int gpio_gecko_port_toggle_bits(const struct device *dev,
				       uint32_t mask)
{
	const struct gpio_gecko_config *config = dev->config;
	GPIO_Port_TypeDef gpio_index = config->gpio_index;

	GPIO_PortOutToggle(gpio_index, mask);

	return 0;
}

static int gpio_gecko_pin_interrupt_configure(const struct device *dev,
					      gpio_pin_t pin,
					      enum gpio_int_mode mode,
					      enum gpio_int_trig trig)
{
	const struct gpio_gecko_config *config = dev->config;
	struct gpio_gecko_data *data = dev->data;

	/* Interrupt on static level is not supported by the hardware */
	if (mode == GPIO_INT_MODE_LEVEL) {
		return -ENOTSUP;
	}

	if (mode == GPIO_INT_MODE_DISABLED) {
		GPIO_IntDisable(BIT(pin));
	} else {
		/* Interrupt line is already in use */
		if ((GPIO->IEN & BIT(pin)) != 0) {
			/* Check if the interrupt is already configured for this port */
			if (!(data->int_enabled_mask & BIT(pin))) {
				return -EBUSY;
			}
		}

		bool rising_edge = true;
		bool falling_edge = true;

		if (trig == GPIO_INT_TRIG_LOW) {
			rising_edge = false;
			falling_edge = true;
		} else if (trig == GPIO_INT_TRIG_HIGH) {
			rising_edge = true;
			falling_edge = false;
		} /* default is GPIO_INT_TRIG_BOTH */

		GPIO_ExtIntConfig(config->gpio_index, pin, pin,
			       rising_edge, falling_edge, true);
	}

	WRITE_BIT(data->int_enabled_mask, pin, mode != GPIO_INT_DISABLE);

	return 0;
}

static int gpio_gecko_manage_callback(const struct device *dev,
				      struct gpio_callback *callback, bool set)
{
	struct gpio_gecko_data *data = dev->data;

	return gpio_manage_callback(&data->callbacks, callback, set);
}

/**
 * Handler for both odd and even pin interrupts
 */
static void gpio_gecko_common_isr(const struct device *dev)
{
	struct gpio_gecko_common_data *data = dev->data;
	uint32_t enabled_int, int_status;
	const struct device *port_dev;
	struct gpio_gecko_data *port_data;

	int_status = GPIO->IF;

	for (unsigned int i = 0; int_status && (i < data->count); i++) {
		port_dev = data->ports[i];
		port_data = port_dev->data;
		enabled_int = int_status & port_data->int_enabled_mask;
		if (enabled_int != 0) {
			int_status &= ~enabled_int;
#if defined(_SILICON_LABS_32B_SERIES_2)
			GPIO->IF_CLR = enabled_int;
#else
			GPIO->IFC = enabled_int;
#endif
			gpio_fire_callbacks(&port_data->callbacks, port_dev,
					    enabled_int);
		}
	}
}
#endif

static DEVICE_API(gpio, gpio_greybush_driver_api) = {
	.pin_configure = gpio_greybush_configure,
#ifdef CONFIG_GPIO_GET_CONFIG
	.pin_get_config = gpio_gecko_get_config,
#endif
#if 0
	.port_get_raw = gpio_gecko_port_get_raw,
	.port_set_masked_raw = gpio_gecko_port_set_masked_raw,
	.port_set_bits_raw = gpio_gecko_port_set_bits_raw,
	.port_clear_bits_raw = gpio_gecko_port_clear_bits_raw,
	.port_toggle_bits = gpio_gecko_port_toggle_bits,
	.pin_interrupt_configure = gpio_gecko_pin_interrupt_configure,
	.manage_callback = gpio_gecko_manage_callback,
#endif
};

static struct greybush_bundle_class_match greybush_gpio_match = {
	.class = 0x0A,
	.protocol = 0x02,
};

static int greybush_gpio_probe(const gb_cport *cport)
{
	return 0;
}

static void greybush_gpio_disconnected(const void *priv)
{
	return;
}

struct gb_driver greybush_class_gpio_driver = {
	.probe = greybush_gpio_probe,
	.disconnected = greybush_gpio_disconnected,
};

// static struct gpio_greybush_class_api uvc_class_api = {
// 	.probe = greybush_gpio_probe,
// 	.disconnected = greybush_gpio_disconnected,
// };
//

GREYBUSH_DEFINE_BUNDLE_CLASS(greybush_gpio_driver,
			     NULL,
			     &greybush_gpio_match);

static int gpio_greybush_init(const struct device *dev)
{
	return 0;
}

#define CONFIG_GREYBUSH_CLASS_GPIO_INSTANCES_COUNT 1
#define CONFIG_GREYBUSH_CLASS_PRIORITY 50

#define GREYBUSH_GPIO_DEVICE_DEFINE(n, _)					\
										\
	static struct gpio_greybush_data gpio_greybush_data_##n = {		\
	};									\
										\
	DEVICE_DEFINE(gpio_greybush_##n, "gpio_greybush_"#n,			\
		      gpio_greybush_init,	NULL,				\
		      &gpio_greybush_data_##n, NULL,				\
		      POST_KERNEL, CONFIG_GREYBUSH_CLASS_PRIORITY,		\
		      &gpio_greybush_driver_api);				\
										\
	GREYBUS_DEFINE_BUNDLE_CLASS(greybus_c_data_##n, &greybush_class_gpio_driver,		\
			  (void *)DEVICE_GET(gpio_greybush_##n),		\
			  &greybush_gpio_match);

LISTIFY(CONFIG_GREYBUSH_CLASS_GPIO_INSTANCES_COUNT, GREYBUSH_GPIO_DEVICE_DEFINE, ())

#if 0
static DEVICE_API(gpio, gpio_gecko_common_driver_api) = {
	.manage_callback = gpio_gecko_manage_callback,
};

static int gpio_gecko_common_init(const struct device *dev);

static const struct gpio_gecko_common_config gpio_gecko_common_config = {
};

static struct gpio_gecko_common_data gpio_gecko_common_data;

DEVICE_DT_DEFINE(DT_INST(0, silabs_gecko_gpio),
		    gpio_gecko_common_init,
		    NULL,
		    &gpio_gecko_common_data, &gpio_gecko_common_config,
		    PRE_KERNEL_1, CONFIG_GPIO_GECKO_COMMON_INIT_PRIORITY,
		    &gpio_gecko_common_driver_api);

static int gpio_gecko_common_init(const struct device *dev)
{
	gpio_gecko_common_data.count = 0;
	IRQ_CONNECT(GPIO_EVEN_IRQn,
		    DT_IRQ_BY_NAME(DT_INST(0, silabs_gecko_gpio), gpio_even, priority),
		    gpio_gecko_common_isr,
		    DEVICE_DT_GET(DT_INST(0, silabs_gecko_gpio)), 0);

	IRQ_CONNECT(GPIO_ODD_IRQn,
		    DT_IRQ_BY_NAME(DT_INST(0, silabs_gecko_gpio), gpio_odd, priority),
		    gpio_gecko_common_isr,
		    DEVICE_DT_GET(DT_INST(0, silabs_gecko_gpio)), 0);

	irq_enable(GPIO_EVEN_IRQn);
	irq_enable(GPIO_ODD_IRQn);

	return 0;
}

#define GPIO_PORT_INIT(idx) \
static int gpio_gecko_port##idx##_init(const struct device *dev); \
\
static const struct gpio_gecko_config gpio_gecko_port##idx##_config = { \
	.common = { \
		.port_pin_mask = (gpio_port_pins_t)(-1), \
	}, \
	.gpio_index = GET_GECKO_GPIO_INDEX(idx), \
}; \
\
static struct gpio_gecko_data gpio_gecko_port##idx##_data; \
\
DEVICE_DT_INST_DEFINE(idx, \
		    gpio_gecko_port##idx##_init, \
		    NULL, \
		    &gpio_gecko_port##idx##_data, \
		    &gpio_gecko_port##idx##_config, \
		    POST_KERNEL, CONFIG_GPIO_INIT_PRIORITY, \
		    &gpio_gecko_driver_api); \
\
static int gpio_gecko_port##idx##_init(const struct device *dev) \
{ \
	gpio_gecko_add_port(&gpio_gecko_common_data, dev); \
	return 0; \
}

DT_INST_FOREACH_STATUS_OKAY(GPIO_PORT_INIT)
#endif
