/*
 * Copyright (c) 2022 Byte Lab d.o.o. <dev@byte-lab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT memsic_mmc5983ma

#include <drivers/spi.h>
#include <init.h>
#include <sys/__assert.h>
#include <sys/byteorder.h>
#include <drivers/sensor.h>
#include <string.h>
#include <logging/log.h>

#include "mmc5983ma.h"

LOG_MODULE_REGISTER(MMC5983MA, CONFIG_SENSOR_LOG_LEVEL);

int mmc5983ma_read_reg(const struct device *dev,
			uint16_t reg_addr, void *data_buf, size_t data_length)
{
	const struct mmc5983ma_config *config = dev->config;
	uint8_t cmd_addr = MMC5983MA_SPI_READ_CMD | reg_addr;
	struct spi_buf buf[2] = {
		{
			.buf = &cmd_addr,
			.len = sizeof(cmd_addr)
		},
		{
			.buf = data_buf,
			.len = data_length
		}
	};

	struct spi_buf_set tx = {
		.buffers = buf,
		/* Read transaction - transfer only header from TX buffer set */
		.count = 1
	};

	struct spi_buf_set rx = {
		.buffers = buf,
		.count = 2
	};

	return spi_transceive_dt(&config->spi, &tx, &rx);
}

int mmc5983ma_write_reg(const struct device *dev,
			uint16_t reg_addr, void *data_buf, size_t data_length)
{
	const struct mmc5983ma_config *config = dev->config;
	uint8_t cmd_addr = MMC5983MA_SPI_WRITE_CMD | reg_addr;
	struct spi_buf buf[2] = {
		{
			.buf = &cmd_addr,
			.len = sizeof(cmd_addr)
		},
		{
			.buf = data_buf,
			.len = data_length
		}
	};

	struct spi_buf_set tx = {
		.buffers = buf,
		.count = 2
	};

	return spi_write_dt(&config->spi, &tx);
}

static void mmc5983ma_convert_magn(struct sensor_value *val, uint32_t raw_val)
{
	int32_t val_extended;

	/* value = (raw_value - 131072) * (1/16384) */
	val_extended = (int32_t) raw_val - 131072;
	val_extended *= 10000;
	val_extended /= 16384;
	val->val1 = val_extended / 10000;
	val->val2 = val_extended % 10000;
}

static void mmc5983ma_convert_temp(struct sensor_value *val, uint32_t raw_val)
{
	int32_t val_extended;

	/* value = (raw_value * 0.8) - 75 */
	val_extended = (int32_t) ((raw_val * 1000u) * 4u / 5u) - (75 * 1000);
	val->val1 = val_extended / 1000u;
	val->val2 = val_extended % 1000u;
}

static int mmc5983ma_channel_get(const struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct mmc5983ma_data *drv_data = dev->data;

	if (chan == SENSOR_CHAN_MAGN_XYZ) {
		mmc5983ma_convert_magn(val, drv_data->x_sample);
		mmc5983ma_convert_magn(val + 1, drv_data->y_sample);
		mmc5983ma_convert_magn(val + 2, drv_data->z_sample);
	} else if (chan == SENSOR_CHAN_MAGN_X) {
		mmc5983ma_convert_magn(val, drv_data->x_sample);
	} else if (chan == SENSOR_CHAN_MAGN_Y) {
		mmc5983ma_convert_magn(val, drv_data->y_sample);
	} else if (chan == SENSOR_CHAN_MAGN_Z) {
		mmc5983ma_convert_magn(val, drv_data->z_sample);
	} else if (chan == SENSOR_CHAN_DIE_TEMP) {
		mmc5983ma_convert_temp(val, drv_data->temp_sample);
	} else {
		LOG_ERR("Invalid sensor channel");
		return -EINVAL;
	}

	return 0;
}

int mmc5983ma_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct mmc5983ma_data *drv_data = dev->data;
	int16_t buf[7];

	if ((chan == SENSOR_CHAN_MAGN_X)
		|| (chan == SENSOR_CHAN_MAGN_Y)
		|| (chan == SENSOR_CHAN_MAGN_Z)
		|| (chan == SENSOR_CHAN_MAGN_XYZ)) {
		/* Magnetometer samples are continuously updated, just read them from registers */
		mmc5983ma_read_reg(dev, MMC5983MA_REG_XOUT0, &buf[0], 7u);

		/* X axis (18-bit) */
		drv_data->x_sample =
			((uint32_t) buf[0] << 10) |
			((uint32_t) buf[1] << 2)  |
			(((uint32_t) buf[6] >> 6) & 0x03);

		/* Y axis (18-bit) */
		drv_data->y_sample =
			((uint32_t) buf[2] << 10) |
			((uint32_t) buf[3] << 2)  |
			(((uint32_t) buf[6] >> 4) & 0x03);

		/* Z axis (18-bit) */
		drv_data->z_sample =
			((uint32_t) buf[4] << 10) |
			((uint32_t) buf[5] << 2)  |
			(((uint32_t) buf[6] >> 2) & 0x03);

	} else if (chan == SENSOR_CHAN_DIE_TEMP) {
		/* Send command to take a die temperature measurement */
		mmc5983ma_read_reg(dev, MMC5983MA_REG_ICTRL0, &buf[0], 1u);
		buf[0] |= MMC5983MA_ICTRL0_TM_T_MASK;
		mmc5983ma_write_reg(dev, MMC5983MA_REG_ICTRL0, &buf[0], 1u);

		/* Wait for die temperature measurement to finish */
		buf[0] = 0;
		while (!(buf[0] & MMC5983MA_STATUS_MEAS_T_DONE_MASK)) {
			mmc5983ma_read_reg(dev, MMC5983MA_REG_STATUS, &buf[0], 1u);
		}

		/* Clear die temperaure measurement done flag */
		buf[0] = MMC5983MA_STATUS_MEAS_T_DONE_MASK;
		mmc5983ma_write_reg(dev, MMC5983MA_REG_STATUS, &buf[0], 1u);

		/* Read die temperature measurement value */
		mmc5983ma_read_reg(dev, MMC5983MA_REG_TOUT, &buf[0], 1u);
		drv_data->temp_sample = buf[0];
	} else {
		return -EINVAL;
	}

	return 0;
}

static const struct sensor_driver_api mmc5983ma_driver_api = {
#if CONFIG_MMC5983MA_TRIGGER
	.trigger_set = mmc5983ma_trigger_set,
#endif
	.sample_fetch = mmc5983ma_sample_fetch,
	.channel_get = mmc5983ma_channel_get,
};

int mmc5983ma_init(const struct device *dev)
{
	const struct mmc5983ma_config *drv_config = dev->config;
	uint8_t buf[4];

	/* Check if SPI is ready */
	if (!device_is_ready(drv_config->spi.bus)) {
		LOG_ERR("SPI dev %s not ready", drv_config->spi.bus->name);
		return -ENODEV;
	}

	/* Do a SW reset of the chip */
	buf[0] = MMC5983MA_ICTRL1_SW_RST_MASK;
	if (mmc5983ma_write_reg(dev, MMC5983MA_REG_ICTRL1, &buf[0], 1) < 0) {
		LOG_ERR("Failed to reset chip");
		return -EIO;
	}

	/* Wait for chip to get out of reset (10 ms according to chip datasheet) */
	k_msleep(10);

	/* Check chip ID */
	if (mmc5983ma_read_reg(dev, MMC5983MA_REG_PRODUCT_ID, &buf[0], 1) < 0) {
		LOG_ERR("Failed to read chip ID");
		return -EIO;
	}

	if (buf[0] != MMC5983MA_CHIP_ID) {
		LOG_ERR("Invalid chip ID! Expected: %x, read: %x", MMC5983MA_CHIP_ID, buf[0]);
		return -EINVAL;
	}

	/* Configure sensor parameters: */
	/* Enable automatic SET operation */
	buf[0] = MMC5983MA_ICTRL0_AUTO_SR_EN_MASK;
	/* Set bandwidth to 800 Hz */
	buf[1] = MMC5983MA_ICTRL1_BW_VAL_800_HZ << MMC5983MA_ICTRL1_BW_SHIFT;
	/* Enable and set ODR and automatic SET frequency according to Kconfig values */
	buf[2] = (MMC5983MA_DEFAULT_ODR << MMC5983MA_ICTRL2_CMM_FREQ_SHIFT)
			| MMC5983MA_ICTRL2_CMM_EN_MASK
			| (MMC5983MA_DEFAULT_PRD_SET << MMC5983MA_ICTRL2_PRD_SET_SHIFT)
			| MMC5983MA_ICTRL2_PRD_SET_EN_MASK;
	/* Use 4 wire SPI */
	buf[3] = 0;
	if (mmc5983ma_write_reg(dev, MMC5983MA_REG_ICTRL0, &buf[0], 4) < 0) {
		LOG_ERR("Failed to configure chip");
		return -EIO;
	}

#ifdef CONFIG_MMC5983MA_TRIGGER
	if (mmc5983ma_init_interrupt(dev) < 0) {
		LOG_ERR("Failed to initialize interrupts");
		return -EIO;
	}
#endif

	return 0;
}

#define MMC5983MA_DEVICE(inst)									\
												\
static struct mmc5983ma_config mmc5983ma_config_##inst = {					\
	.spi		= SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),	\
	.int_gpio	= COND_CODE_1(DT_INST_NODE_HAS_PROP(inst, int_gpios),			\
					(GPIO_DT_SPEC_INST_GET(inst, int_gpios)), ({ 0 })),	\
};												\
												\
static struct mmc5983ma_data mmc5983ma_data_##inst;						\
												\
DEVICE_DT_INST_DEFINE(inst,									\
			&mmc5983ma_init,							\
			NULL,									\
			&mmc5983ma_data_##inst,							\
			&mmc5983ma_config_##inst,						\
			POST_KERNEL,								\
			CONFIG_SENSOR_INIT_PRIORITY,						\
			&mmc5983ma_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MMC5983MA_DEVICE)
