/*
 * Copyright (c) 2023 Tomislav Milkovic
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT aosong_aht

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
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
static const uint8_t aht_cmd_load_calibration[] = { 0xBEu, 0x08u, 0x00u };
static const uint8_t aht_cmd_start_measurement[] = { 0xACu, 0x33u, 0x00u };


static bool aht_check_crc(uint8_t *buf)
{
	/* Uses Dallas/Maxim CRC-8 polynomial */
	if(buf[6] != crc8(&buf[0], 6, 0x31, 0xFF, false))
	{
		LOG_ERR("Measurement data CRC mismatch! Calculated: %x, received: %x", crc8(&buf[0], 6, 0x31, 0xFF, false), buf[6]);
		LOG_HEXDUMP_ERR(buf, 7, "Registers: ");
		return false;
	}

	return true;
}

static int aht_reset(const struct device *dev)
{
	const struct aht_config *config = dev->config;
	int ret = 0;

	/* Toggle power supply GPIO if power-gpios property was provided in DT */
	if(device_is_ready(config->power_gpio.port)) {
		ret = gpio_pin_configure_dt(&config->power_gpio, GPIO_OUTPUT_INACTIVE);
		k_msleep(100);
		ret = gpio_pin_set_dt(&config->power_gpio, 1);
	} else {
		/* Otherwise send soft reset command */
		ret = i2c_write_dt(&config->bus, &aht_cmd_soft_reset[0], ARRAY_SIZE(aht_cmd_soft_reset));
		if(ret < 0) {
			LOG_ERR("Failed to do a soft reset");
			return ret;
		}
	}

	return 0;
}

static int aht_load_calibration_values(const struct device *dev)
{
	const struct aht_config *config = dev->config;
	int ret = 0;

	ret = i2c_write_dt(&config->bus, &aht_cmd_load_calibration[0], ARRAY_SIZE(aht_cmd_load_calibration));
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

	return i2c_write_dt(&config->bus, &aht_cmd_start_measurement[0], ARRAY_SIZE(aht_cmd_start_measurement));
}

static int aht_read_status(const struct device *dev, uint8_t* status)
{
	const struct aht_config *config = dev->config;
	uint8_t status_cmd = 0x71u;

	return i2c_write_read_dt(&config->bus, &status_cmd, sizeof(status_cmd), status, sizeof(*status));
	/*return i2c_read_dt(&config->bus, status, 1);*/
}

static int aht_sample_fetch(const struct device *dev,
			    enum sensor_channel chan)
{
	const struct aht_config *config = dev->config;
	struct aht_data *data = dev->data;
	uint8_t rx_buf[7];
	int ret;

	__ASSERT_NO_MSG(chan == SENSOR_CHAN_ALL);
	
	/* Start measurement */
	ret = aht_start_measurement(dev);
	if(ret < 0) {
		LOG_ERR("Failed to start measurement, error code: %d", ret);
		return ret;
	}

	/* Wait for measurement to complete */
	k_msleep(AHT_MEASUREMENT_DELAY_MS);

	/* Read device status */
	ret = aht_read_status(dev, &rx_buf[0]);
	if(ret < 0) {
		LOG_ERR("Failed to read device status, error code: %d", ret);
		return ret;
	}

	/* Check if measurement is done */
	if((rx_buf[0] & AHT_STATUS_BUSY) != 0) {
		LOG_ERR("Measurement timeout");
		return -EIO;
	}
	
	/* Read measurement data */
	ret = i2c_read_dt(&config->bus, &rx_buf[0], ARRAY_SIZE(rx_buf));
	if(ret < 0) {
		LOG_ERR("Failed to read measurement data");
		return ret;
	}

	/* Check if message CRC is correct */
	if(!aht_check_crc(&rx_buf[0])) {
		LOG_WRN("Measurement data CRC mismatch");
	}

	/* Store raw samples */
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
		val->val2 = (((int32_t)(tmp >> 20u) - 50000) % 1000) * 1000;
	} else if(chan == SENSOR_CHAN_HUMIDITY) {
		/* humidity = rh_sample * 100 / 2^20 */
		tmp = (uint64_t)data->rh_sample * 100u * 1000u;
		val->val1 = (uint32_t)(tmp >> 20u) / 1000u;
		val->val2 = ((uint32_t)(tmp >> 20u) % 1000u) * 1000;
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
	uint8_t status = 0;
	int ret = 0;

	k_msleep(40);

	if(!device_is_ready(config->bus.bus)) {
		LOG_ERR("I2C bus %s is not ready!", config->bus.bus->name);
		return -EINVAL;
	}

	/* Do a chip reset */
	ret = aht_reset(dev);
	if(ret < 0) {
		LOG_ERR("Failed to reset chip, error code: %d", ret);
		return ret;
	}

	/* Wait for chip to boot up */
	k_msleep(AHT_RESET_DELAY_MS);

	/* Read device status */
	ret = aht_read_status(dev, &status);
	if(ret < 0) {
		LOG_ERR("Failed to read device status, error code: %d", ret);
		return ret;
	}

	if((status & AHT_STATUS_CALIBRATED) != AHT_STATUS_CALIBRATED)
	{
		ret = aht_load_calibration_values(dev);
		if(ret < 0) {
			LOG_ERR("Failed to load calibration values, error code: %d", ret);
			return ret;
		}
	}
	
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
