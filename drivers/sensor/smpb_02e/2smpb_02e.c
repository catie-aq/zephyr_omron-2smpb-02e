/*
 * Copyright (c) 2024, CATIE
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT omron_2smpb_02e

#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>

#include "2smpb_02e.h"

LOG_MODULE_REGISTER(O2SMPB_02E, CONFIG_SENSOR_LOG_LEVEL);

struct _2smpb_02e_config {
	struct i2c_dt_spec i2c;
};

struct _2smpb_02e_data {
};

static int _2smpb_02e_attr_set(const struct device *dev, enum sensor_channel chan,
			       enum sensor_attribute attr, const struct sensor_value *val)
{
	return 0;
}

static int _2smpb_02e_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct _2smpb_02e_data *data = dev->data;
	const struct _2smpb_02e_config *config = dev->config;

	return 0;
}

static int _2smpb_02e_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct _2smpb_02e_data *data = dev->data;

	// TODO: Update val with the sensor value
	val->val1 = 0;
	val->val2 = 0;

	return 0;
}

static int _2smpb_02e_init(const struct device *dev)
{
	const struct _2smpb_02e_config *config = dev->config;
	struct _2smpb_02e_data *data = dev->data;
	uint8_t chip_id;

	// Reset the sensor
	i2c_reg_write_byte_dt(&config->i2c, O2SMPB_02_REG_RESET, 0xE6);

	k_sleep(K_MSEC(10));

	// Read CHIP ID register to make sure the device is present
	i2c_reg_read_byte_dt(&config->i2c, O2SMPB_02_REG_CHIP_ID, &chip_id);

	if (chip_id != 0x5C) {
		LOG_ERR("Invalid chip ID");
		return -EIO;
	}

	return 0;
}

static const struct sensor_driver_api _2smpb_02e_driver_api = {
	.attr_set = _2smpb_02e_attr_set,
	.sample_fetch = _2smpb_02e_sample_fetch,
	.channel_get = _2smpb_02e_channel_get,
};

#define O2SMPB_02E_INIT(n)                                                                         \
	static struct _2smpb_02e_config _2smpb_02e_config_##n = {                                  \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
	};                                                                                         \
	static struct _2smpb_02e_data _2smpb_02e_data_##n;                                         \
	DEVICE_DT_INST_DEFINE(n, _2smpb_02e_init, NULL, &_2smpb_02e_data_##n,                      \
			      &_2smpb_02e_config_##n, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,    \
			      &_2smpb_02e_driver_api);

DT_INST_FOREACH_STATUS_OKAY(O2SMPB_02E_INIT)
