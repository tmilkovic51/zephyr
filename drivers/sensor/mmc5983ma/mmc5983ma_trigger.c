/*
 * Copyright (c) 2022 Byte Lab d.o.o. <dev@byte-lab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT memsic_mmc5983ma

#include <zephyr/device.h>
#include <zephyr/sys/__assert.h>
#include <zephyr/sys/util.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "mmc5983ma.h"

LOG_MODULE_DECLARE(MMC5983MA, CONFIG_SENSOR_LOG_LEVEL);

int mmc5983ma_trigger_set(const struct device *dev,
			const struct sensor_trigger *trig,
			sensor_trigger_handler_t handler)
{
	struct mmc5983ma_data *drv_data = dev->data;
	int8_t buf;

	__ASSERT_NO_MSG(trig->type == SENSOR_TRIG_DATA_READY);

	if ((handler == NULL) || (trig == NULL)) {
		return -EIO;
	}

	drv_data->data_ready_handler = handler;
	drv_data->data_ready_trigger = *trig;

	/* Clear IRQ flags (if any) by writing to MMC5983MA status register */
	buf = MMC5983MA_STATUS_MEAS_M_DONE_MASK | MMC5983MA_STATUS_MEAS_T_DONE_MASK;
	if (mmc5983ma_write_reg(dev, MMC5983MA_REG_STATUS, &buf, sizeof(buf) < 0)) {
		LOG_ERR("Could not clear interrupt flags");
		return -EIO;
	}

	/* Read the previous value of ICTRL0 register */
	if (mmc5983ma_read_reg(dev, MMC5983MA_REG_ICTRL0, &buf, sizeof(buf) < 0)) {
		LOG_ERR("Could not read ICTRL0 register");
		return -EIO;
	}

	/* Set INT_MEAS_DONE_EN bit in ICTRL0 register */
	buf |= MMC5983MA_ICTRL0_MEAS_INT_EN_MASK;
	if (mmc5983ma_write_reg(dev, MMC5983MA_REG_ICTRL0, &buf, sizeof(buf) < 0)) {
		LOG_ERR("Could not enable the measurement done interrupt");
		return -EIO;
	}

	return 0;
}

static void mmc5983ma_gpio_callback(const struct device *dev,
				  struct gpio_callback *cb, uint32_t pins)
{
	struct mmc5983ma_data *drv_data =
		CONTAINER_OF(cb, struct mmc5983ma_data, gpio_cb);
	const struct device *mmc5983ma_dev = drv_data->dev;
	const struct mmc5983ma_config *drv_config = mmc5983ma_dev->config;

	ARG_UNUSED(dev);
	ARG_UNUSED(pins);

	/* Turn off GPIO interrupt until IRQ flags are cleared on MMC5983MA by thread */
	gpio_pin_interrupt_configure_dt(&drv_config->int_gpio,
			GPIO_INT_DISABLE);

#if defined(CONFIG_MMC5983MA_TRIGGER_OWN_THREAD)
	k_sem_give(&drv_data->gpio_sem);
#elif defined(CONFIG_MMC5983MA_TRIGGER_GLOBAL_THREAD)
	k_work_submit(&drv_data->work);
#endif
}

static void mmc5983ma_thread_cb(const struct device *dev)
{
	struct mmc5983ma_data *drv_data = dev->data;
	const struct mmc5983ma_config *drv_config = dev->config;
	uint8_t buf;

	/* Clear IRQ flags by writing to MMC5983MA status register */
	buf = MMC5983MA_STATUS_MEAS_M_DONE_MASK | MMC5983MA_STATUS_MEAS_T_DONE_MASK;
	if (mmc5983ma_write_reg(dev, MMC5983MA_REG_STATUS, &buf, sizeof(buf) < 0)) {
		LOG_ERR("Could not clear interrupt flags");
		return;
	}

	if (drv_data->data_ready_handler != NULL) {
		drv_data->data_ready_handler(dev,
					     &drv_data->data_ready_trigger);
	}

	/* IRQ flags on MMC5983MA have been cleared, re-enable GPIO interrupt */
	gpio_pin_interrupt_configure_dt(&drv_config->int_gpio,
			GPIO_INT_LEVEL_ACTIVE);
}

#ifdef CONFIG_MMC5983MA_TRIGGER_OWN_THREAD
static void mmc5983ma_thread(struct mmc5983ma_data *drv_data)
{
	while (1) {
		k_sem_take(&drv_data->gpio_sem, K_FOREVER);
		mmc5983ma_thread_cb(drv_data->dev);
	}
}
#endif

#ifdef CONFIG_MMC5983MA_TRIGGER_GLOBAL_THREAD
static void mmc5983ma_work_cb(struct k_work *work)
{
	struct mmc5983ma_data *drv_data =
		CONTAINER_OF(work, struct mmc5983ma_data, work);

	mmc5983ma_thread_cb(drv_data->dev);
}
#endif

int mmc5983ma_init_interrupt(const struct device *dev)
{
	struct mmc5983ma_data *drv_data = dev->data;
	const struct mmc5983ma_config *drv_config = dev->config;
	uint8_t buf;

	if (drv_config->int_gpio.port == NULL) {
		LOG_ERR("Interrupt request GPIO not specified in device tree");
		return -EINVAL;
	}

	gpio_pin_configure_dt(&drv_config->int_gpio, GPIO_INPUT);

	gpio_init_callback(&drv_data->gpio_cb,
			   mmc5983ma_gpio_callback,
			   BIT(drv_config->int_gpio.pin));

	if (gpio_add_callback(drv_config->int_gpio.port, &drv_data->gpio_cb) < 0) {
		LOG_ERR("Could not set GPIO callback");
		return -EIO;
	}

	/* Clear IRQ flags by writing to MMC5983MA status register */
	buf = MMC5983MA_STATUS_MEAS_M_DONE_MASK | MMC5983MA_STATUS_MEAS_T_DONE_MASK;
	if (mmc5983ma_write_reg(dev, MMC5983MA_REG_STATUS, &buf, sizeof(buf) < 0)) {
		LOG_ERR("Could not clear interrupt flags");
		return -EIO;
	}

	drv_data->dev = dev;

#if defined(CONFIG_MMC5983MA_TRIGGER_OWN_THREAD)
	k_sem_init(&drv_data->gpio_sem, 0, K_SEM_MAX_LIMIT);

	k_thread_create(&drv_data->thread, drv_data->thread_stack,
			CONFIG_MMC5983MA_THREAD_STACK_SIZE,
			(k_thread_entry_t) mmc5983ma_thread, drv_data,
			NULL, NULL, K_PRIO_COOP(CONFIG_MMC5983MA_THREAD_PRIORITY),
			0, K_NO_WAIT);
#elif defined(CONFIG_MMC5983MA_TRIGGER_GLOBAL_THREAD)
	drv_data->work.handler = mmc5983ma_work_cb;
#endif

	gpio_pin_interrupt_configure_dt(&drv_config->int_gpio,
			GPIO_INT_LEVEL_ACTIVE);

	return 0;
}
