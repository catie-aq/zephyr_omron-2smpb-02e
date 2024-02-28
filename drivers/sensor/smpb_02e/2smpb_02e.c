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

#define COEFFICIENT_A1_A  -6.3E-03
#define COEFFICIENT_A1_S  4.3E-04
#define COEFFICIENT_A2_A  -1.9E-11
#define COEFFICIENT_A2_S  1.2E-10
#define COEFFICIENT_BT1_A 1.0E-01
#define COEFFICIENT_BT1_S 9.1E-02
#define COEFFICIENT_BT2_A 1.2E-08
#define COEFFICIENT_BT2_S 1.2E-06
#define COEFFICIENT_BP1_A 3.3E-02
#define COEFFICIENT_BP1_S 1.9E-02
#define COEFFICIENT_B11_A 2.1E-07
#define COEFFICIENT_B11_S 1.4E-07
#define COEFFICIENT_BP2_A -6.3E-10
#define COEFFICIENT_BP2_S 3.5E-10
#define COEFFICIENT_B12_A 2.9E-13
#define COEFFICIENT_B12_S 7.6E-13
#define COEFFICIENT_B21_A 2.1E-15
#define COEFFICIENT_B21_S 1.2E-14
#define COEFFICIENT_BP3_A 1.3E-16
#define COEFFICIENT_BP3_S 7.9E-17

#define U20TOS32(x) (-(x & 0x00080000) + (x & 0xFFF7FFFF))
#define U16TOS16(x) (-(x & 0x8000) | (x & 0x7FFF))

struct o2smpb_02e_config {
	struct i2c_dt_spec i2c;
};

struct o2smpb_02e_data {
	int32_t b00;
	int32_t a0;
	float bt1;
	float bp1;
	float bt2;
	float b11;
	float bp2;
	float b12;
	float b21;
	float bp3;
	float a1;
	float a2;
	int32_t dt;
	int32_t dp;
};

static int o2smpb_02e_read_coefficients(const struct device *dev)
{
	struct o2smpb_02e_data *data = dev->data;
	const struct o2smpb_02e_config *config = dev->config;
	uint8_t buffer[25];

	if (i2c_burst_read_dt(&config->i2c, O2SMPB_02_REG_COE_b00_1, buffer, sizeof(buffer)) < 0) {
		LOG_ERR("Failed to read coefficients");
		return -EIO;
	}

	// K = OTP / 16
	data->a0 =
		(int32_t)U20TOS32((buffer[18] << 12 | buffer[19] << 4 | (buffer[24] & 0x0F))) >> 4;
	data->b00 =
		(int32_t)U20TOS32((buffer[0] << 12 | buffer[1] << 4 | (buffer[24] & 0xF0) >> 4)) >>
		4;

	// K = A + (S * OTP) / 32768
	int16_t bt1 = U16TOS16((buffer[2] << 8 | buffer[3]));
	data->bt1 = COEFFICIENT_BT1_A + ((COEFFICIENT_BT1_S * bt1) / 32768.0);

	int16_t bp1 = U16TOS16((buffer[6] << 8 | buffer[7]));
	data->bp1 = COEFFICIENT_BP1_A + ((COEFFICIENT_BP1_S * bp1) / 32768.0);

	int16_t bt2 = U16TOS16((buffer[4] << 8 | buffer[5]));
	data->bt2 = COEFFICIENT_BT2_A + ((COEFFICIENT_BT2_S * bt2) / 32768.0);

	int16_t b11 = U16TOS16((buffer[8] << 8 | buffer[9]));
	data->b11 = COEFFICIENT_B11_A + ((COEFFICIENT_B11_S * b11) / 32768.0);

	int16_t bp2 = U16TOS16((buffer[10] << 8 | buffer[11]));
	data->bp2 = COEFFICIENT_BP2_A + ((COEFFICIENT_BP2_S * bp2) / 32768.0);

	int16_t b12 = U16TOS16((buffer[12] << 8 | buffer[13]));
	data->b12 = COEFFICIENT_B12_A + ((COEFFICIENT_B12_S * b12) / 32768.0);

	int16_t b21 = U16TOS16((buffer[14] << 8 | buffer[15]));
	data->b21 = COEFFICIENT_B21_A + ((COEFFICIENT_B21_S * b21) / 32768.0);

	int16_t bp3 = U16TOS16((buffer[16] << 8 | buffer[17]));
	data->bp3 = COEFFICIENT_BP3_A + ((COEFFICIENT_BP3_S * bp3) / 32768.0);

	int16_t a1 = (int16_t)U16TOS16((buffer[20] << 8 | buffer[21]));
	data->a1 = COEFFICIENT_A1_A + ((COEFFICIENT_A1_S * a1) / 32768.0);

	int16_t a2 = U16TOS16((buffer[22] << 8 | buffer[23]));
	a2 = COEFFICIENT_A2_A + ((COEFFICIENT_A2_S * a2) / 32768.0);

	return 0;
}

static int o2smpb_02e_attr_set(const struct device *dev, enum sensor_channel chan,
			       enum sensor_attribute attr, const struct sensor_value *val)
{
	return 0;
}

static int o2smpb_02e_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct o2smpb_02e_data *data = dev->data;
	const struct o2smpb_02e_config *config = dev->config;

	uint8_t buffer[3];

	// Force mode
	if (i2c_reg_write_byte_dt(&config->i2c, O2SMPB_02_REG_CTRL_MEAS,
				  (uint8_t)(0x101 << 5 | 0x101 << 2 | 0x1)) < 0) {
		LOG_ERR("Could not set sensor to force mode");
		return -EIO;
	}

	k_sleep(K_MSEC(500));

	if (i2c_burst_read_dt(&config->i2c, O2SMPB_02_REG_TEMP_TXD2, buffer, sizeof(buffer)) < 0) {
		LOG_ERR("Could not read sensor data");
		return -EIO;
	}

	data->dt = (buffer[0] << 16 | buffer[1] << 8 | buffer[2]) - (0x1 << 23);

	if (i2c_burst_read_dt(&config->i2c, O2SMPB_02_REG_PRESS_TXD2, buffer, sizeof(buffer)) < 0) {
		LOG_ERR("Could not read sensor data");
		return -EIO;
	}

	data->dp = (buffer[0] << 16 | buffer[1] << 8 | buffer[2]) - (0x1 << 23);

	return 0;
}

static int o2smpb_02e_channel_get(const struct device *dev, enum sensor_channel chan,
				  struct sensor_value *val)
{
	struct o2smpb_02e_data *data = dev->data;

	switch (chan) {
	case SENSOR_CHAN_AMBIENT_TEMP:
		double temp = (data->a0) + ((data->a1) + (data->a2) * data->dt) * data->dt;
		temp /= 256.0;

		val->val1 = (int32_t)temp;
		val->val2 = (int32_t)((temp - val->val1) * 1000000);
		break;
	case SENSOR_CHAN_PRESS:
		double tr = (data->a0) + ((data->a1) + (data->a2) * data->dt) * data->dt;

		double press = data->b00 + data->bt1 * tr + data->bp1 * (float)data->dp +
			       data->b11 * data->dp * tr + data->bt2 * tr * tr +
			       data->bp2 * data->dp * data->dp + data->b12 * data->dp * tr * tr +
			       data->b21 * data->dp * data->dp * tr +
			       data->bp3 * data->dp * data->dp * data->dp;
		press /= 100.0;

		val->val1 = (int32_t)press;
		val->val2 = (int32_t)((press - val->val1) * 1000000);
		break;
	default:
		return -ENOTSUP;
		return 0;
	}

	return 0;
}

static int o2smpb_02e_init(const struct device *dev)
{
	const struct o2smpb_02e_config *config = dev->config;
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

	if (o2smpb_02e_read_coefficients(dev) < 0) {
		return -EIO;
	}

	return 0;
}

static const struct sensor_driver_api o2smpb_02e_driver_api = {
	.attr_set = o2smpb_02e_attr_set,
	.sample_fetch = o2smpb_02e_sample_fetch,
	.channel_get = o2smpb_02e_channel_get,
};

#define O2SMPB_02E_INIT(n)                                                                         \
	static struct o2smpb_02e_config o2smpb_02e_config_##n = {                                  \
		.i2c = I2C_DT_SPEC_INST_GET(n),                                                    \
	};                                                                                         \
	static struct o2smpb_02e_data o2smpb_02e_data_##n;                                         \
	DEVICE_DT_INST_DEFINE(n, o2smpb_02e_init, NULL, &o2smpb_02e_data_##n,                      \
			      &o2smpb_02e_config_##n, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,    \
			      &o2smpb_02e_driver_api);

DT_INST_FOREACH_STATUS_OKAY(O2SMPB_02E_INIT)
