/*
 * Based on sdmmc_stm32,c:
 * Copyright (c) 2020 Amarula Solutions.
 *
 * eMMC updates:
 * Copyright (c) 2021 Logitech, Inc
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT st_stm32_emmc

#include <zephyr/devicetree.h>
#include <zephyr/drivers/disk.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/stm32_clock_control.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <soc.h>
#include <stm32_ll_rcc.h>

LOG_MODULE_REGISTER(stm32_emmc, CONFIG_SDMMC_LOG_LEVEL);

typedef void (*irq_config_func_t)(const struct device *dev);

struct stm32_sdmmc_priv {
	irq_config_func_t irq_config;
	struct k_sem thread_lock;
	struct k_sem sync;
	MMC_HandleTypeDef hsd;
	int status;
	const struct gpio_dt_spec rst_gpio;
	struct stm32_pclken pclken;
	const struct pinctrl_dev_config *pcfg;
};

#ifdef CONFIG_SDMMC_STM32_HWFC
static void stm32_sdmmc_fc_enable(struct stm32_sdmmc_priv *priv)
{
	MMC_TypeDef *sdmmcx = priv->hsd.Instance;

	sdmmcx->CLKCR |= SDMMC_CLKCR_HWFC_EN;
}
#endif

static void stm32_sdmmc_isr(const struct device *dev)
{
	struct stm32_sdmmc_priv *priv = dev->data;

	HAL_MMC_IRQHandler(&priv->hsd);
}

void HAL_MMC_TxCpltCallback(MMC_HandleTypeDef *hsd)
{
	struct stm32_sdmmc_priv *priv = CONTAINER_OF(hsd, struct stm32_sdmmc_priv, hsd);

	priv->status = hsd->ErrorCode;

	k_sem_give(&priv->sync);
}

void HAL_MMC_RxCpltCallback(MMC_HandleTypeDef *hsd)
{
	struct stm32_sdmmc_priv *priv = CONTAINER_OF(hsd, struct stm32_sdmmc_priv, hsd);

	priv->status = hsd->ErrorCode;

	k_sem_give(&priv->sync);
}

void HAL_MMC_ErrorCallback(MMC_HandleTypeDef *hsd)
{
	struct stm32_sdmmc_priv *priv = CONTAINER_OF(hsd, struct stm32_sdmmc_priv, hsd);

	priv->status = hsd->ErrorCode;

	k_sem_give(&priv->sync);
}

static int stm32_sdmmc_clock_enable(struct stm32_sdmmc_priv *priv)
{
	const struct device *clock;

#if CONFIG_SOC_SERIES_STM32L4X
	LL_RCC_PLLSAI1_Disable();
	/* Configure PLLSA11 to enable 48M domain */
	LL_RCC_PLLSAI1_ConfigDomain_48M(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLLM_DIV_1, 8,
					LL_RCC_PLLSAI1Q_DIV_8);

	/* Enable PLLSA1 */
	LL_RCC_PLLSAI1_Enable();

	/*  Enable PLLSAI1 output mapped on 48MHz domain clock */
	LL_RCC_PLLSAI1_EnableDomain_48M();

	/* Wait for PLLSA1 ready flag */
	while (LL_RCC_PLLSAI1_IsReady() != 1)
		;

	LL_RCC_SetSDMMCClockSource(LL_RCC_SDMMC1_CLKSOURCE_PLLSAI1);
#endif

	clock = DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE);

	/* Enable the APB clock for stm32_sdmmc */
	return clock_control_on(clock, (clock_control_subsys_t *)&priv->pclken);
}

static int stm32_emmc_access_init(struct disk_info *disk)
{
	const struct device *dev = disk->dev;
	struct stm32_sdmmc_priv *priv = dev->data;
	int err;

	if (priv->status == DISK_STATUS_OK) {
		return 0;
	}

	if (priv->status == DISK_STATUS_NOMEDIA) {
		return -ENODEV;
	}

	err = stm32_sdmmc_clock_enable(priv);
	if (err) {
		LOG_ERR("failed to init clocks");
		return err;
	}

	err = HAL_MMC_Init(&priv->hsd);
	if (err != HAL_OK) {
		LOG_ERR("failed to init stm32_emmc");
		return -EIO;
	}

#ifdef CONFIG_SDMMC_STM32_HWFC
	stm32_sdmmc_fc_enable(priv);
#endif

	priv->status = DISK_STATUS_OK;
	return 0;
}

static int stm32_emmc_access_status(struct disk_info *disk)
{
	const struct device *dev = disk->dev;
	struct stm32_sdmmc_priv *priv = dev->data;

	return priv->status;
}

static int stm32_emmc_access_read(struct disk_info *disk, uint8_t *data_buf, uint32_t start_sector,
				  uint32_t num_sector)
{
	const struct device *dev = disk->dev;
	struct stm32_sdmmc_priv *priv = dev->data;
	int err;

	k_sem_take(&priv->thread_lock, K_FOREVER);

	err = HAL_MMC_ReadBlocks_IT(&priv->hsd, data_buf, start_sector, num_sector);
	if (err != HAL_OK) {
		LOG_ERR("sd read block failed %d", err);
		err = -EIO;
		goto end;
	}

	k_sem_take(&priv->sync, K_FOREVER);

	if (priv->status != DISK_STATUS_OK) {
		LOG_ERR("sd read error %d", priv->status);
		err = -EIO;
		goto end;
	}

	while (HAL_MMC_GetCardState(&priv->hsd) != HAL_MMC_CARD_TRANSFER) {
	}

end:
	k_sem_give(&priv->thread_lock);
	return err;
}

static int stm32_emmc_access_write(struct disk_info *disk, const uint8_t *data_buf,
				   uint32_t start_sector, uint32_t num_sector)
{
	const struct device *dev = disk->dev;
	struct stm32_sdmmc_priv *priv = dev->data;
	int err;

	k_sem_take(&priv->thread_lock, K_FOREVER);

	err = HAL_MMC_WriteBlocks_IT(&priv->hsd, (uint8_t *)data_buf, start_sector, num_sector);
	if (err != HAL_OK) {
		LOG_ERR("sd write block failed %d", err);
		err = -EIO;
		goto end;
	}

	k_sem_take(&priv->sync, K_FOREVER);

	if (priv->status != DISK_STATUS_OK) {
		LOG_ERR("sd write error %d", priv->status);
		err = -EIO;
		goto end;
	}

	while (HAL_MMC_GetCardState(&priv->hsd) != HAL_MMC_CARD_TRANSFER) {
	}

end:
	k_sem_give(&priv->thread_lock);
	return err;
}

static int stm32_emmc_access_ioctl(struct disk_info *disk, uint8_t cmd, void *buff)
{
	const struct device *dev = disk->dev;
	struct stm32_sdmmc_priv *priv = dev->data;
	HAL_MMC_CardInfoTypeDef info;
	int err;

	switch (cmd) {
	case DISK_IOCTL_GET_SECTOR_COUNT:
		err = HAL_MMC_GetCardInfo(&priv->hsd, &info);
		if (err != HAL_OK) {
			return -EIO;
		}
		*(uint32_t *)buff = info.LogBlockNbr;
		break;
	case DISK_IOCTL_GET_SECTOR_SIZE:
		err = HAL_MMC_GetCardInfo(&priv->hsd, &info);
		if (err != HAL_OK) {
			return -EIO;
		}
		*(uint32_t *)buff = info.LogBlockSize;
		break;
	case DISK_IOCTL_GET_ERASE_BLOCK_SZ:
		*(uint32_t *)buff = 1;
		break;
	case DISK_IOCTL_CTRL_SYNC:
		/* we use a blocking API, so nothing to do for sync */
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static const struct disk_operations stm32_emmc_ops = {
	.init = stm32_emmc_access_init,
	.status = stm32_emmc_access_status,
	.read = stm32_emmc_access_read,
	.write = stm32_emmc_access_write,
	.ioctl = stm32_emmc_access_ioctl,
};

static struct disk_info stm32_emmc_info = {
	.name = CONFIG_SDMMC_VOLUME_NAME,
	.ops = &stm32_emmc_ops,
};

static int stm32_emmc_reset_init(struct stm32_sdmmc_priv *priv)
{
	int err = 0;

	if (!priv->rst_gpio.port) {
		return 0;
	}

	/* Configure GPIO as output and assert RST pin */
	err = gpio_pin_configure_dt(&priv->rst_gpio, GPIO_OUTPUT_ACTIVE);
	if (err) {
		return err;
	}

	k_sleep(K_MSEC(50));

	/* Deassert RST pin after 50 ms */
	err = gpio_pin_set_dt(&priv->rst_gpio, 0);

	return err;
}

static int stm32_emmc_reset_uninit(struct stm32_sdmmc_priv *priv)
{
	if (!priv->rst_gpio.port) {
		return 0;
	}

	gpio_pin_configure_dt(&priv->rst_gpio, GPIO_DISCONNECTED);
	return 0;
}

static int disk_stm32_emmc_init(const struct device *dev)
{
	struct stm32_sdmmc_priv *priv = dev->data;
	int err;

	/* Configure dt provided device signals when available */
	err = pinctrl_apply_state(priv->pcfg, PINCTRL_STATE_DEFAULT);
	if (err < 0) {
		return err;
	}

	priv->irq_config(dev);

	/* Initialize semaphores */
	k_sem_init(&priv->thread_lock, 1, 1);
	k_sem_init(&priv->sync, 0, 1);

	err = stm32_emmc_reset_init(priv);
	if (err) {
		return err;
	}

	stm32_emmc_info.dev = dev;
	err = disk_access_register(&stm32_emmc_info);
	if (err) {
		goto err_reset;
	}
	return 0;

err_reset:
	stm32_emmc_reset_uninit(priv);
	return err;
}

#if DT_NODE_HAS_STATUS(DT_DRV_INST(0), okay)

PINCTRL_DT_INST_DEFINE(0);

static void stm32_sdmmc_irq_config_func(const struct device *dev)
{
	IRQ_CONNECT(DT_INST_IRQN(0), DT_INST_IRQ(0, priority), stm32_sdmmc_isr,
		    DEVICE_DT_INST_GET(0), 0);
	irq_enable(DT_INST_IRQN(0));
}

static struct stm32_sdmmc_priv stm32_sdmmc_priv_1 = {
	.irq_config = stm32_sdmmc_irq_config_func,
	.hsd = {
		.Instance = (SDMMC_TypeDef *)DT_INST_REG_ADDR(0),
	},
	.rst_gpio = GPIO_DT_SPEC_INST_GET_OR(0, reset_gpios, { 0 }),
	.pclken = {
		.bus = DT_INST_CLOCKS_CELL(0, bus),
		.enr = DT_INST_CLOCKS_CELL(0, bits),
	},
	.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};

DEVICE_DT_INST_DEFINE(0, disk_stm32_emmc_init, NULL, &stm32_sdmmc_priv_1, NULL, POST_KERNEL,
		      CONFIG_SDMMC_INIT_PRIORITY, NULL);
#endif
