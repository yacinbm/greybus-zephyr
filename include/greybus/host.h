
#ifndef _GREYBUS_HOST_H
#define _GREYBUS_HOST_H

/**
 * @brief Bundle matching structure.
 */
struct greybush_bundle_class_match {
	uint8_t class;
	uint8_t protocol;
};


struct greybus_device {
	uint16_t cport;
};

/**
 * @brief Greybus host class instance data
 */
struct greybush_class_data {
	/** GB Driver to use */
	struct gb_driver *api;

	/** Pointer to private data */
	void *priv;
};

struct greybush_class_node {
	struct greybush_class_data *const c_data;

	/** Filter rules to match this USB host class instance against a device class **/
	const struct greybush_bundle_class_match *filter;
};

/**
 * @brief Define Greybus host support bundle match
 *
 */
#define GREYBUSH_DEFINE_BUNDLE_CLASS(bundle_name, bundle_api, _priv, _filter)	\
	static struct greybush_class_data UTIL_CAT(class_data_, bundle_name) = {	\
		.api = &bundle_drv,						\
		.priv = _priv,							\
	};									\
	static STRUCT_SECTION_ITERABLE(greybush_class_node, bundle_name) = {	\
		.c_data = &UTIL_CAT(class_data, bundle_name),			\
		.filter = _filter,						\
	};


#endif
