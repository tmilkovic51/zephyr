/*
 * Copyright (c) 2023 Tomislav Milkovic
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aosong_aht

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(AHT, CONFIG_SENSOR_LOG_LEVEL);


#define AHT_RESET_DELAY_MS		(100u)
#define AHT_MEASUREMENT_DELAY_MS	(80u)

#define AHT_STATUS_CALIBRATED		BIT(3)
#define AHT_STATUS_BUSY			BIT(7)

struct aht_config {
	struct i2c_dt_spec bus;
	struct gpio_dt_spec power_gpio;
};

struct aht_data {
	uint32_t t_sample;
	uint32_t rh_sample;
};

static const uint8_t aht_cmd_soft_reset[] = { 0xBAu };
static const uint8_t aht_cmd_load_calibration[] = { 0xBEu, 0x08u, 0x00u }; /* 0xE1 for AHT1x series, 0xB1 (or maybe 0xBE?) for AHT2x series */
static const uint8_t aht_cmd_start_measurement[] = { 0xACu, 0x33u, 0x00u };


static bool aht_check_crc(uint8_t *buf)
{
	/* Uses Dallas/Maxim CRC-8 polynomial */
	return (buf[6] == crc8(&buf[0], 6, 0x31, 0xFF, false));
}

static int aht_reset(const struct device *dev)
{
	const struct aht_config *config = dev->config;

	/* Toggle power supply GPIO if power-gpios property was provided in DT */
	if(device_is_ready(config->power_gpio.port)) {
		gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_INACTIVE);
		k_msleep(10);
		gpio_pin_set_dt(&config->power_gpio, 1);
	} else {
		/* Otherwise send soft reset command */
		if(i2c_write_dt(&config->bus, &aht_cmd_soft_reset[0], sizeof(aht_cmd_soft_reset)) < 0) {
			LOG_ERR("Failed to do a soft reset");
			return -EIO;
		}
	}

	/* Wait for chip to boot up */
	k_msleep(AHT_RESET_DELAY_MS);

	return 0;
}

static int aht_load_calibration_values(const struct device *dev)
{
	const struct aht_config *config = dev->config;
	int ret = 0;

	ret = i2c_write_dt(&config->bus, &aht_cmd_load_calibration[0], sizeof(aht_cmd_load_calibration));
	if(ret < 0) {
		LOG_ERR("Failed to load calibration values");
		return ret;
	}
	
	k_msleep(10);

	return ret;
}

static int aht_start_measurement(const struct device *dev)
{
	const struct aht_config *config = dev->config;
	int ret = 0;

	ret = i2c_write_dt(&config->bus, &aht_cmd_start_measurement[0], sizeof(aht_cmd_start_measurement));
	if(ret < 0) {
		LOG_ERR("Failed to start measurement");
	}

	return ret;
}

static uint8_t aht_read_status(const struct device *dev)
{
	const struct aht_config *config = dev->config;
	uint8_t rx_buf;

	if(i2c_read_dt(&config->bus, &rx_buf, sizeof(rx_buf)) < 0) {
		/* Fail-safe value on failed read */
		rx_buf = AHT_STATUS_BUSY;
		LOG_ERR("Failed to read device status");
	}

	return rx_buf;
}

static int aht_sample_fetch(const struct device *dev,
			    enum sensor_channel chan)
{
	const struct aht_config *config = dev->config;
	struct aht_data *data = dev->data;
	uint8_t rx_buf[7];

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
	
	aht_start_measurement(dev);
	
	k_msleep(AHT_MEASUREMENT_DELAY_MS);
	
	/* Check measurement done */
	rx_buf[0] = aht_read_status(dev);
	if((rx_buf[0] & AHT_STATUS_BUSY) != 0) {
		/* Measurement timeout */
		return -EIO;
	}
	
	/* Read measurement data */
	if(i2c_read_dt(&config->bus, &rx_buf[0], sizeof(rx_buf)) < 0) {
		LOG_ERR("Failed to read measurement data");
		return -EIO;
	}
	
	if(!aht_check_crc(&rx_buf[0])) {
		LOG_ERR("Measurement data CRC mismatch");
		return -EIO;
	}

	data->rh_sample = (rx_buf[1] << 12) | (rx_buf[2] << 4) | ((rx_buf[3] >> 4) & 0x0F);
	data->t_sample = ((rx_buf[3] & 0x0F) << 16) | (rx_buf[4] << 8) | rx_buf[5];

	return 0;
}

static int aht_channel_get(const struct device *dev,
			   enum sensor_channel chan,
			   struct sensor_value *val)
{
	const struct aht_data *data = dev->data;
	uint64_t tmp;

	if(chan == SENSOR_CHAN_AMBIENT_TEMP) {
		/* temperature = (t_sample * 200 / 2^20) - 50 */
		tmp = (uint64_t)data->t_sample * 200u * 1000u;
		val->val1 = ((int32_t)(tmp >> 20u) - 50000) / 1000;
		val->val2 = ((int32_t)(tmp >> 20u) - 50000) % 1000;
	} else if(chan == SENSOR_CHAN_HUMIDITY) {
		/* humidity = rh_sample * 100 / 2^20 */
		tmp = (uint64_t)data->rh_sample * 100u * 1000u;
		val->val1 = (uint32_t)(tmp >> 20u) / 1000u;
		val->val2 = (uint32_t)(tmp >> 20u) % 1000u;
	} else {
		return -ENOTSUP;
	}

	return 0;
}

static const struct sensor_driver_api aht_driver_api = {
	.sample_fetch = aht_sample_fetch,
	.channel_get = aht_channel_get,
};

static int aht_init(const struct device *dev)
{
	const struct aht_config *config = dev->config;
	int ret = 0;

	if(!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C bus %s is not ready!", config->bus.bus->name);
		return -EINVAL;
	}
	
	ret = aht_reset(dev);
	
	ret = aht_load_calibration_values(dev);

	return 0;
}

#define AHT_DEFINE(inst)							\
	struct aht_data aht_data_##inst;					\
	static const struct aht_config aht_cfg_##inst = {			\
		.bus = I2C_DT_SPEC_INST_GET(inst),				\
		.power_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, power_gpios, {0}),	\
	};									\
										\
	SENSOR_DEVICE_DT_INST_DEFINE(inst, aht_init, NULL,			\
		&aht_data_##inst, &aht_cfg_##inst,				\
		POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,			\
		&aht_driver_api);

DT_INST_FOREACH_STATUS_OKAY(AHT_DEFINE)
