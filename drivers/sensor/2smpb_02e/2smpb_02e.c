/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT omron_2smpb_02e

#include <zephyr/logging/log.h>

#include "2smpb_02e.h"

LOG_MODULE_REGISTER(2SMPB_02E, CONFIG_SENSOR_LOG_LEVEL);

struct 2smpb_02e_config {
};

struct 2smpb_02e_data {
};

static int 2smpb_02e_attr_set(const struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr, const struct sensor_value *val)
{
	return 0;
}

static int 2smpb_02e_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct 2smpb_02e_data *dev_data = dev->data;
	const struct 2smpb_02e_cfg *dev_cfg = dev->config;

	return 0;
}

static int 2smpb_02e_channel_get(const struct device *dev, enum sensor_channel chan,
			      struct sensor_value *val)
{
	struct 2smpb_02e_data *dev_data = dev->data;

    // TODO: Update val with the sensor value
	val->val1 = 0;
	val->val2 = 0;

	return 0;
}

static int 2smpb_02e_init(const struct device *dev)
{
	const struct 2smpb_02e_cfg *cfg = dev->config;
	struct 2smpb_02e_data *dev_data = dev->data;

	return 0;
}

static const struct sensor_driver_api 2smpb_02e_driver_api = {
	.attr_set = 2smpb_02e_attr_set,
	.sample_fetch = 2smpb_02e_sample_fetch,
	.channel_get = 2smpb_02e_channel_get,
};

#define 2SMPB_02E_INIT(n)                                                                             \
	static struct 2smpb_02e_cfg 2smpb_02e_config_##n = {                                             \
	};                                                                                         \
	static struct 2smpb_02e_data 2smpb_02e_data_##n;                                                 \
	DEVICE_DT_INST_DEFINE(n, 2smpb_02e_init, NULL, &2smpb_02e_data_##n, &2smpb_02e_config_##n,          \
			      POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, &2smpb_02e_driver_api);

DT_INST_FOREACH_STATUS_OKAY(2SMPB_02E_INIT)
