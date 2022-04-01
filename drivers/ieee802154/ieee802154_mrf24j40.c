/* ieee802154_mrf24j40.c - Microchip MRF24J40 driver */

#define DT_DRV_COMPAT microchip_mrf24j40

/*
 * Copyright (c) 2022 Byte-Lab d.o.o. <dev@byte-lab.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ieee802154_mrf24j40, CONFIG_IEEE802154_DRIVER_LOG_LEVEL);

#include "ieee802154_mrf24j40_regs.h"

#include <errno.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/debug/stack.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/init.h>
#include <zephyr/kernel/thread_stack.h>
#include <zephyr/net/ieee802154_radio.h>
#include <zephyr/net/net_core.h>
#include <zephyr/net/net_if.h>
#include <zephyr/net/net_pkt.h>
#include <zephyr/pm/device.h>
#include <zephyr/random/rand32.h>
#include <zephyr/sys/byteorder.h>

#define MRF24J40_FCS_LENGTH  (2)
#define MRF24J40_PSDU_LENGTH (127)

#define MRF24J40_OUTPUT_POWER_MAX (36)
#define MRF24J40_OUTPUT_POWER_MIN (0)

/* Default settings for device initialization */
#define MRF24J40_DEFAULT_TX_POWER (0)
#define MRF24J40_DEFAULT_CHANNEL  (26)

struct mrf24j40_config {
	struct spi_dt_spec bus;
	struct gpio_dt_spec int_gpio;
	struct gpio_dt_spec reset_gpio;
	struct gpio_dt_spec wake_gpio;
	bool has_mac;
	bool nonbeacon_mode_en;
	bool turbo_mode_en;
	bool sleep_clock_32khz_en;
	bool pa_lna_ctrl_en;
	uint8_t gpio3_output;
	uint8_t gpio4_output;
	uint8_t gpio5_output;
};

struct mrf24j40_data {
	const struct device *dev;
	struct net_if *iface;
	uint8_t mac_addr[8];
	struct gpio_callback int_cb;

	struct k_mutex phy_mutex;
	struct k_sem isr_sem;
	struct k_sem tx_sem;
	int tx_status;

	K_KERNEL_STACK_MEMBER(mrf24j40_thread_stack, CONFIG_IEEE802154_MRF24J40_THREAD_STACK_SIZE);
	struct k_thread mrf24j40_thread;
};

/*
  Read from long address space register/FIFO or from short address space
  register, depending if the most significant bit of address is set/cleared
*/
static int mrf24j40_read(const struct device *dev, uint16_t addr, uint8_t *data_buf,
			 size_t data_length)
{
	const struct mrf24j40_config *config = dev->config;
	uint8_t cmd_buf[2];
	size_t cmd_buf_len;

	if (addr & MRF24J40_LONG_ADDR_SPECIFIER_MASK) {
		/* Put address to buffer in big endian format (MSB goes first) */
		sys_put_be16(addr, cmd_buf);
		cmd_buf_len = 2u;
	} else {
		/* 8-bit short address is left-justified */
		cmd_buf[0] = (addr >> 8u) & 0xFFu;
		cmd_buf_len = 1u;
	}

	const struct spi_buf bufs[2] = {{.buf = cmd_buf, .len = cmd_buf_len},
					{.buf = data_buf, .len = data_length}};

	const struct spi_buf_set tx = {.buffers = bufs, .count = 1};
	const struct spi_buf_set rx = {.buffers = bufs, .count = 2};

	return spi_transceive_dt(&config->bus, &tx, &rx);
}

/*
  Write to long address space register/FIFO or to short address space
  register, depending if the most significant bit of address is set/cleared
*/
static int mrf24j40_write(const struct device *dev, uint16_t addr, uint8_t *data_buf,
			  size_t data_length)
{
	const struct mrf24j40_config *config = dev->config;
	uint8_t cmd_buf[2];
	size_t cmd_buf_len;

	if (addr & MRF24J40_LONG_ADDR_SPECIFIER_MASK) {
		addr |= MRF24J40_LONG_ADDR_WRITE_MASK;
		/* Put address to buffer in big endian format (MSB goes first) */
		sys_put_be16(addr, cmd_buf);
		cmd_buf_len = 2u;
	} else {
		addr |= MRF24J40_SHORT_ADDR_WRITE_MASK;
		/* 8-bit short address is left-justified */
		cmd_buf[0] = (addr >> 8u) & 0xFFu;
		cmd_buf_len = 1u;
	}

	const struct spi_buf buf_tx[2] = {{.buf = cmd_buf, .len = cmd_buf_len},
					  {.buf = data_buf, .len = data_length}};

	const struct spi_buf_set tx = {.buffers = &buf_tx[0], .count = 2};

	return spi_write_dt(&config->bus, &tx);
}

/* Update register bit-field */
static int mrf24j40_update_reg(const struct device *dev, uint16_t addr, uint8_t mask, uint8_t val)
{
	int err = 0;
	uint8_t tmp;

	/* Clear bits in input value that are not part of the mask */
	val &= mask;

	/* Read previous value */
	err = mrf24j40_read(dev, addr, &tmp, sizeof(tmp));
	if (err != 0) {
		return err;
	}

	/* Clear masked bits in previous value */
	tmp &= ~mask;

	/* Set masked bits according to input value */
	tmp |= val;

	return mrf24j40_write(dev, addr, &tmp, sizeof(tmp));
}

static inline int mrf24j40_mcu_pin_interrupt_mask(const struct device *dev, bool enable)
{
	const struct mrf24j40_config *config = dev->config;
	gpio_flags_t flags = enable ? GPIO_INT_LEVEL_ACTIVE : GPIO_INT_DISABLE;

	return gpio_pin_interrupt_configure_dt(&config->int_gpio, flags);
}

/* Enable or disable (enable = `false`) all interrupts from asserting INT pin */
static int mrf24j40_chip_interrupts_mask(const struct device *dev, bool enable)
{
	uint8_t buf;

	if (enable) {
		/* Writing 0 to bit enables the interrupt */
		buf = ~((uint8_t)(MRF24J40_REG_INTCON_TXNIE | MRF24J40_REG_INTCON_RXIE |
				  MRF24J40_REG_INTCON_SECIE));
	} else {
		/* Writing 1 to bit disables the interrupt */
		buf = MRF24J40_REG_INTCON_DISABLE_ALL_MASK;
	}

	return mrf24j40_write(dev, MRF24J40_REG_ADDR_INTCON, &buf, sizeof(buf));
}

static int mrf24j40_rssi_raw_to_dbm(uint8_t rssi)
{
	int16_t dbm;

	/* Convert raw RSSI value to RSSI value in dBm - graph is linear (datasheet, figure 3-3) */
	dbm = -90 + (0.2156862745f * rssi);

	return dbm;
}

static inline int mrf24j40_read_rxfifo_content(const struct device *dev, struct net_pkt *pkt,
					       uint8_t len)
{
	struct net_buf *buf = pkt->buffer;
	uint8_t lqi;
	uint8_t rssi_raw;
	int16_t rssi_dbm;
	int err = 0;

	/* Read packet data */
	err = mrf24j40_read(dev, MRF24J40_FIFO_ADDR_RX_DATA, buf->data, len);
	if (err == 0) {
		net_buf_add(buf, len);

		/* Read LQI and RSSI */
		err = mrf24j40_read(dev, MRF24J40_FIFO_ADDR_RX_LQI(len), &lqi, sizeof(lqi));
		err |= mrf24j40_read(dev, MRF24J40_FIFO_ADDR_RX_RSSI(len), &rssi_raw, sizeof(rssi_raw));
		rssi_dbm = mrf24j40_rssi_raw_to_dbm(rssi_raw);

		net_pkt_set_ieee802154_lqi(pkt, lqi);
		net_pkt_set_ieee802154_rssi_dbm(pkt, rssi_dbm);

		LOG_DBG("Caught an IEEE 802.15.4 packet (size: %u bytes, LQI: %u, RSSI: %d dBm)", len,
			lqi, rssi_dbm);
	}

	/* Flush RX FIFO (MRF24J40 Silicon Errata, issue #1) */
	err |= mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_RXFLUSH, MRF24J40_REG_RXFLUSH_RXFLUSH,
				   MRF24J40_REG_RXFLUSH_RXFLUSH);

	return err;
}

static inline int mrf24j40_transmit_frame(const struct device *dev, struct net_pkt *pkt,
					  struct net_buf *frag)
{
	uint16_t fifo_frame_len_addr;
	uint8_t fifo_frame_len;
	uint16_t fifo_frame_data_addr;
	uint8_t *fifo_frame_data;
	uint16_t fifo_control_reg_addr;
	uint8_t fifo_control_reg;
	int err = 0;

	if (frag->len > MRF24J40_PSDU_LENGTH) {
		LOG_ERR("Frame too long - requested: %u bytes, maximum: %u bytes", frag->len,
			MRF24J40_PSDU_LENGTH);
		return -EINVAL;
	}

	fifo_frame_len = (uint8_t)frag->len;
	fifo_frame_data = frag->data;

	if ((*fifo_frame_data & 0x07u) == 0u) {
		/*
		 * Beacon frame - prepare addresses
		 * and values for transmitting from beacon FIFO
		 */
		fifo_frame_len_addr = MRF24J40_FIFO_ADDR_BEACON_TX_FRAME_LEN;
		fifo_frame_data_addr = MRF24J40_FIFO_ADDR_BEACON_TX_DATA;
		fifo_control_reg_addr = MRF24J40_REG_ADDR_TXBCON0;

		/* Set the trigger transmission bit */
		fifo_control_reg = MRF24J40_REG_TXBCON0_TXBTRIG;
	} else {
		/*
		 * Data / MAC command / ACK frame - prepare addresses
		 * and values for transmitting from normal TX FIFO
		 */
		fifo_frame_len_addr = MRF24J40_FIFO_ADDR_NORMAL_TX_FRAME_LEN;
		fifo_frame_data_addr = MRF24J40_FIFO_ADDR_NORMAL_TX_DATA;
		fifo_control_reg_addr = MRF24J40_REG_ADDR_TXNCON;

		/* Set the trigger transmission bit */
		fifo_control_reg = MRF24J40_REG_TXNCON_TXNTRIG;

		/* If frame needs to be acknowledged, also set the TXNACKREQ bit */
		if (ieee802154_is_ar_flag_set(frag)) {
			fifo_control_reg |= MRF24J40_REG_TXNCON_TXNACKREQ;
		}
	}

	/* Write frame length (header length + payload length) to FIFO */
	err = mrf24j40_write(dev, fifo_frame_len_addr, &fifo_frame_len, sizeof(fifo_frame_len));

	/* Write frame data to FIFO */
	err |= mrf24j40_write(dev, fifo_frame_data_addr, fifo_frame_data, fifo_frame_len);

	/* Trigger the transmission */
	err |= mrf24j40_write(dev, fifo_control_reg_addr, &fifo_control_reg,
			      sizeof(fifo_control_reg));

	return err;
}

static int mrf24j40_rf_state_machine_reset(const struct device *dev)
{
	uint8_t buf;
	int err = 0;

	/* Assert RF state machine reset */
	buf = MRF24J40_REG_RFCTL_RFRST;
	err = mrf24j40_write(dev, MRF24J40_REG_ADDR_RFCTL, &buf, sizeof(buf));
	if (err != 0) {
		return err;
	}
	/* Deassert RF state machine reset */
	buf = 0;
	err = mrf24j40_write(dev, MRF24J40_REG_ADDR_RFCTL, &buf, sizeof(buf));
	if (err != 0) {
		return err;
	}
	/* Delay for at least 192 us to allow the RF circuitry to calibrate after reset */
	k_busy_wait(200u);

	return err;
}

static int mrf24j40_hardware_reset(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	int err = 0;

	/* Do a hardware reset if RESET pin is defined in DT */
	if (config->reset_gpio.port) {
		/* Assert the RESET pin */
		err = gpio_pin_set_dt(&config->reset_gpio, 1);
		if (err != 0) {
			return err;
		}
		k_msleep(1);
		/* Release the RESET pin */
		err = gpio_pin_set_dt(&config->reset_gpio, 0);
		if (err != 0) {
			return err;
		}
		/* Wait 2 ms for RF circuitry to start up and stabilize */
		k_msleep(2);
	}

	return err;
}

static int mrf24j40_software_reset(const struct device *dev, bool reset_mac, bool reset_baseband,
				   bool reset_power)
{
	uint8_t buf;
	uint8_t mask = 0;
	int err = 0;

	if (reset_mac) {
		mask |= MRF24J40_REG_SOFTRST_RSTMAC;
	}
	if (reset_baseband) {
		mask |= MRF24J40_REG_SOFTRST_RSTBB;
	}
	if (reset_power) {
		mask |= MRF24J40_REG_SOFTRST_RSTPWR;
	}

	/* Do a soft reset of selected circuitry (MAC, baseband or power circuitry) */
	buf = mask;
	err = mrf24j40_write(dev, MRF24J40_REG_ADDR_SOFTRST, &buf, sizeof(buf));
	if (err != 0) {
		return err;
	}

	/* Wait until soft reset flags are automatically cleared */
	do {
		err = mrf24j40_read(dev, MRF24J40_REG_ADDR_SOFTRST, &buf, sizeof(buf));
		if (err != 0) {
			return err;
		}
	} while (buf & mask);

	return err;
}

static int mrf24j40_read_interrupt_status_flags(const struct device *dev, uint8_t *flags)
{
	int err = 0;

	/* Interrupt status flags are cleared upon reading the INTSTAT register */
	if (flags) {
		err = mrf24j40_read(dev, MRF24J40_REG_ADDR_INTSTAT, flags, sizeof(*flags));
	}

	return err;
}

static int mrf24j40_turbo_mode_enable(const struct device *dev)
{
	int err = 0;
	uint8_t buf;

	/* Enable turbo mode */
	buf = MRF24J40_REG_BBREG0_TURBO;
	err = mrf24j40_write(dev, MRF24J40_REG_ADDR_BBREG0, &buf, sizeof(buf));
	if (err != 0) {
		return err;
	}

	/* Set PREVALIDTH to recommended value for turbo mode */
	buf = (0x03 << MRF24J40_REG_BBREG3_PREVALIDTH_SHIFT) & MRF24J40_REG_BBREG3_PREVALIDTH_MASK;
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_BBREG3,
				  MRF24J40_REG_BBREG3_PREVALIDTH_MASK, buf);
	if (err != 0) {
		return err;
	}

	/* Set CSTH to recommended value for turbo mode */
	buf = (0x02 << MRF24J40_REG_BBREG4_CSTH_SHIFT) & MRF24J40_REG_BBREG4_CSTH_MASK;
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_BBREG4, MRF24J40_REG_BBREG4_CSTH_MASK,
				  buf);
	if (err != 0) {
		return err;
	}

	/* Perform a baseband circuitry reset */
	err = mrf24j40_software_reset(dev, false, true, false);
	if (err != 0) {
		return err;
	}

	/* Reset RF state machine */
	return mrf24j40_rf_state_machine_reset(dev);
}

static inline void mrf24j40_get_mac(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	struct mrf24j40_data *data = dev->data;
	uint32_t *ptr = (uint32_t *)(data->mac_addr);

	if (!config->has_mac) {
		UNALIGNED_PUT(sys_rand32_get(), ptr);
		ptr = (uint32_t *)(data->mac_addr + 4);
		UNALIGNED_PUT(sys_rand32_get(), ptr);
	}

	/*
	 * Clear bit 0 to ensure it isn't a multicast address and set
	 * bit 1 to indicate address is locally administered and may
	 * not be globally unique.
	 */
	data->mac_addr[0] = (data->mac_addr[0] & ~0x01) | 0x02;
}

static inline void mrf24j40_handle_tx_done(const struct device *dev)
{
	struct mrf24j40_data *data = dev->data;
	uint8_t buf;

	/* Read transmission status register */
	mrf24j40_read(dev, MRF24J40_REG_ADDR_TXSTAT, &buf, sizeof(buf));

	/* Set TX status and give TX semaphore so transmitting thread can unblock */
	if (((buf & MRF24J40_REG_TXSTAT_TXNSTAT) != 0)) {
		LOG_ERR("Transmission failed after %u retries",
			((buf & MRF24J40_REG_TXSTAT_TXNRETRY_MASK) >>
			 MRF24J40_REG_TXSTAT_TXNRETRY_SHIFT));
		if (((buf & MRF24J40_REG_TXSTAT_CCAFAIL) != 0)) {
			LOG_ERR("Channel was busy (CSMA-CA time out)");
		}
		data->tx_status = -ETIMEDOUT;
	} else {
		LOG_DBG("Transmission successful");
		data->tx_status = 0;
	}

	k_sem_give(&data->tx_sem);
}

static inline void mrf24j40_handle_rx(const struct device *dev)
{
	struct mrf24j40_data *data = dev->data;
	struct net_pkt *pkt = NULL;
	uint8_t pkt_len;

	/* Steps to read RX FIFO: datasheet chapter 3.11, example 3-2 */

	/* Set RXDECINV bit to disable packet reception while RXFIFO is being read */
	mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_BBREG1, MRF24J40_REG_BBREG1_RXDECINV,
			    MRF24J40_REG_BBREG1_RXDECINV);

	/* Read frame length (frame length = header + payload + FCS) */
	if (mrf24j40_read(dev, MRF24J40_FIFO_ADDR_RX_FRAME_LEN, &pkt_len, 1u)) {
		LOG_ERR("Unable to read frame length");
		goto out;
	}

	/* Copy FCS into packet only if raw mode or OpenThread is used */
	if (!IS_ENABLED(CONFIG_IEEE802154_RAW_MODE) && !IS_ENABLED(CONFIG_NET_L2_OPENTHREAD)) {
		pkt_len -= MRF24J40_FCS_LENGTH;
	}

	/* Allocate packet buffer for the received frame */
	pkt = net_pkt_alloc_with_buffer(data->iface, pkt_len, AF_UNSPEC, 0, K_NO_WAIT);
	if (!pkt) {
		LOG_ERR("No buf available");
		goto out;
	}

	/* Read the frame from RX FIFO into packet buffer */
	if (mrf24j40_read_rxfifo_content(dev, pkt, pkt_len)) {
		LOG_ERR("No content read");
		goto out;
	}

	if (ieee802154_handle_ack(data->iface, pkt) == NET_OK) {
		LOG_DBG("ACK packet handled");
		goto out;
	}

	/* Pass the packet to L2 layer */
	if (net_recv_data(data->iface, pkt) < 0) {
		LOG_DBG("Packet dropped by NET stack");
		goto out;
	}

	log_stack_usage(&data->mrf24j40_thread);

	/* Clear RXDECINV bit to re-enable packet reception */
	mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_BBREG1, MRF24J40_REG_BBREG1_RXDECINV, 0);

	return;

out:
	if (pkt) {
		net_pkt_unref(pkt);
	}

	/* Clear RXDECINV bit to re-enable packet reception */
	mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_BBREG1, MRF24J40_REG_BBREG1_RXDECINV, 0);
}

static void mrf24j40_thread_main(void *arg)
{
	const struct device *dev = arg;
	struct mrf24j40_data *data = dev->data;
	uint8_t int_status;
	int err = 0;

	while (true) {
		k_sem_take(&data->isr_sem, K_FOREVER);

		/*
		 * Host MCU GPIO interrupt has been disabled at this point
		 * (this is done in ISR before k_sem_give())
		 */

		k_mutex_lock(&data->phy_mutex, K_FOREVER);

		/* Read the INTSTAT register - interrupt status flags are cleared by reading it */
		if (mrf24j40_read_interrupt_status_flags(dev, &int_status)) {
			LOG_ERR("Failed to read INTSTAT register");
			goto unmask_irq;
		}

		/* Check if RX interrupt happened */
		if (int_status & MRF24J40_REG_INTSTAT_RXIF) {
			mrf24j40_handle_rx(dev);
		}

		/* Check if TX interrupt happened */
		if (int_status & MRF24J40_REG_INTSTAT_TXNIF) {
			mrf24j40_handle_tx_done(dev);
		}

		/* Check if secured frame has been received (security bit in frame header set) */
		if (int_status & MRF24J40_REG_INTSTAT_SECIF) {
			/* Unsecuring the frame not supported yet, just flush the RX FIFO */
			err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_RXFLUSH,
						  MRF24J40_REG_RXFLUSH_RXFLUSH,
						  MRF24J40_REG_RXFLUSH_RXFLUSH);
		}

	unmask_irq:
		/*
		 * Re-enable host MCU interrupts - if new IRQ happened in the meantime,
		 * ISR will give the semaphore again and it will be served immediately
		 */
		if (mrf24j40_mcu_pin_interrupt_mask(dev, true)) {
			LOG_ERR("Failed to unmask MCU interrupt");
		}

		k_mutex_unlock(&data->phy_mutex);
	}
}

static inline void mrf24j40_int_handler(const struct device *port, struct gpio_callback *cb,
					uint32_t pins)
{
	struct mrf24j40_data *data = CONTAINER_OF(cb, struct mrf24j40_data, int_cb);

	/*
	 * Disable host MCU interrupt - interrupt is level-driven and it will be re-enabled
	 * by the driver thread (which waits on a semaphore) after interrupt has been served
	 */
	mrf24j40_mcu_pin_interrupt_mask(data->dev, false);
	k_sem_give(&data->isr_sem);
}

static enum ieee802154_hw_caps mrf24j40_get_capabilities(const struct device *dev)
{
	return IEEE802154_HW_FCS | IEEE802154_HW_PROMISC | IEEE802154_HW_FILTER |
	       IEEE802154_HW_CSMA | IEEE802154_HW_2_4_GHZ | IEEE802154_HW_TX_RX_ACK |
	       IEEE802154_HW_RX_TX_ACK | IEEE802154_HW_ENERGY_SCAN;
}

static int mrf24j40_cca(const struct device *dev)
{
	/*
	 * Note: Doing CCA explicitly is not supported - CCA is done automatically
	 * by HW before every transmission (except when transmitting ACK and beacon
	 * frames, which is also done automatically by HW)
	 */

	ARG_UNUSED(dev);

	return 0;
}

static int mrf24j40_set_channel(const struct device *dev, uint16_t channel)
{
	struct mrf24j40_data *data = dev->data;
	uint8_t buf;
	int ret = 0;

	if ((channel < IEEE802154_2_4_GHZ_CHANNEL_MIN) ||
	    (channel > IEEE802154_2_4_GHZ_CHANNEL_MAX)) {
		LOG_ERR("Unsupported channel %u", channel);
		return -EINVAL;
	}

	k_mutex_lock(&data->phy_mutex, K_FOREVER);

	/* Disable all interrupts */
	ret = mrf24j40_chip_interrupts_mask(dev, false);
	if (ret != 0) {
		LOG_ERR("Failed to disable interrupts");
	}

	if (ret == 0) {
		/* Change channel (only channels 11 - 26 are supported) */
		buf = (channel - IEEE802154_2_4_GHZ_CHANNEL_MIN)
		      << MRF24J40_REG_RFCON0_CHANNEL_SHIFT;
		ret = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_RFCON0,
					  MRF24J40_REG_RFCON0_CHANNEL_MASK, buf);
		if (ret != 0) {
			LOG_ERR("Failed to set channel %u", channel);
		}
	}

	if (ret == 0) {
		/* RF state machine should be reset after channel change */
		ret = mrf24j40_rf_state_machine_reset(dev);
		if (ret != 0) {
			LOG_ERR("Failed to reset the RF state machine");
		}
	}

	/* Re-enable all interrupts */
	ret = mrf24j40_chip_interrupts_mask(dev, true);
	if (ret != 0) {
		LOG_ERR("Failed to enable interrupts");
	}

	k_mutex_unlock(&data->phy_mutex);

	return ret;
}

static int mrf24j40_set_pan_id(const struct device *dev, uint16_t pan_id)
{
	struct mrf24j40_data *data = dev->data;
	uint16_t pan_id_buf = sys_cpu_to_le16(pan_id);
	int ret = 0;

	k_mutex_lock(&data->phy_mutex, K_FOREVER);

	ret = mrf24j40_write(dev, MRF24J40_REG_ADDR_PANIDL, (uint8_t *)&pan_id_buf,
			     sizeof(pan_id_buf));
	if (ret != 0) {
		LOG_ERR("Failed setting PAN ID %x", pan_id);
	}

	k_mutex_unlock(&data->phy_mutex);

	return ret;
}

static int mrf24j40_set_short_addr(const struct device *dev, uint16_t short_addr)
{
	struct mrf24j40_data *data = dev->data;
	uint16_t short_addr_buf = sys_cpu_to_le16(short_addr);
	int ret = 0;

	k_mutex_lock(&data->phy_mutex, K_FOREVER);

	ret = mrf24j40_write(dev, MRF24J40_REG_ADDR_SADRL, (uint8_t *)&short_addr_buf,
			     sizeof(short_addr_buf));
	if (ret != 0) {
		LOG_ERR("Failed setting short address %x", short_addr);
	}

	k_mutex_unlock(&data->phy_mutex);

	return ret;
}

static int mrf24j40_set_ieee_addr(const struct device *dev, const uint8_t *ieee_addr)
{
	struct mrf24j40_data *data = dev->data;
	int ret = 0;

	k_mutex_lock(&data->phy_mutex, K_FOREVER);

	ret = mrf24j40_write(dev, MRF24J40_REG_ADDR_EADR0, (uint8_t *)ieee_addr, 8u);
	if (ret != 0) {
		LOG_ERR("Failed setting IEEE address %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x",
			ieee_addr[7], ieee_addr[6], ieee_addr[5], ieee_addr[4], ieee_addr[3],
			ieee_addr[2], ieee_addr[1], ieee_addr[0]);
	}

	k_mutex_unlock(&data->phy_mutex);

	return 0;
}

static int mrf24j40_filter(const struct device *dev, bool set, enum ieee802154_filter_type type,
			   const struct ieee802154_filter *filter)
{
	if (!set) {
		return -ENOTSUP;
	}

	LOG_DBG("Applying filter %u", type);

	if (type == IEEE802154_FILTER_TYPE_IEEE_ADDR) {
		return mrf24j40_set_ieee_addr(dev, filter->ieee_addr);
	} else if (type == IEEE802154_FILTER_TYPE_SHORT_ADDR) {
		return mrf24j40_set_short_addr(dev, filter->short_addr);
	} else if (type == IEEE802154_FILTER_TYPE_PAN_ID) {
		return mrf24j40_set_pan_id(dev, filter->pan_id);
	}

	return -ENOTSUP;
}

static int mrf24j40_set_txpower(const struct device *dev, int16_t dbm)
{
	struct mrf24j40_data *data = dev->data;
	uint8_t buf;
	uint8_t large_scale;
	uint8_t small_scale;
	int ret = 0;

	if ((dbm > MRF24J40_OUTPUT_POWER_MAX) || (dbm < MRF24J40_OUTPUT_POWER_MIN)) {
		return -EINVAL;
	}

	large_scale = (dbm / 10) << MRF24J40_REG_RFCON3_TXPWRL_SHIFT;
	buf = large_scale;
	small_scale = dbm % 10;

	if (small_scale > 6) {
		buf |= 0x07 << MRF24J40_REG_RFCON3_TXPWRS_SHIFT;
	} else if (small_scale > 4) {
		buf |= 0x06 << MRF24J40_REG_RFCON3_TXPWRS_SHIFT;
	} else if (small_scale > 3) {
		buf |= 0x05 << MRF24J40_REG_RFCON3_TXPWRS_SHIFT;
	} else if (small_scale > 2) {
		buf |= 0x04 << MRF24J40_REG_RFCON3_TXPWRS_SHIFT;
	} else if (small_scale > 1) {
		buf |= 0x03 << MRF24J40_REG_RFCON3_TXPWRS_SHIFT;
	} else if (small_scale > 0) {
		buf |= 0x02 << MRF24J40_REG_RFCON3_TXPWRS_SHIFT;
	}

	k_mutex_lock(&data->phy_mutex, K_FOREVER);

	ret = mrf24j40_write(dev, MRF24J40_REG_ADDR_RFCON3, &buf, sizeof(buf));
	if (ret != 0) {
		LOG_ERR("Failed to set TX output power of %d dBm", dbm);
	}

	k_mutex_unlock(&data->phy_mutex);
	return ret;
}

static int mrf24j40_tx(const struct device *dev, enum ieee802154_tx_mode mode, struct net_pkt *pkt,
		       struct net_buf *frag)
{
	struct mrf24j40_data *data = dev->data;
	int ret_val = 0;

	/* CSMA-CA cannot be turned off (it is handled in HW) */
	if (mode != IEEE802154_TX_MODE_CSMA_CA) {
		LOG_ERR("TX mode %d not supported", mode);
		ret_val = -ENOTSUP;
	}

	if (ret_val == 0) {
		LOG_DBG("Transmitting frame with length: %u)", frag->len);

		k_mutex_lock(&data->phy_mutex, K_FOREVER);

		/* Transmit the frame */
		ret_val = mrf24j40_transmit_frame(dev, pkt, frag);

		k_mutex_unlock(&data->phy_mutex);

		if (ret_val == 0) {
			/* Wait for transmission to complete */
			k_sem_take(&data->tx_sem, K_FOREVER);
			ret_val = data->tx_status;
		} else {
			LOG_ERR("Failed writing frame to device's TX buffer");
		}
	}

	return ret_val;
}

static int mrf24j40_start(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	struct mrf24j40_data *data = dev->data;
	uint8_t testmode = MRF24J40_REG_TESTMODE_NORMAL_MODE_VAL;
	int err = 0;

	k_mutex_lock(&data->phy_mutex, K_FOREVER);

	err = mrf24j40_mcu_pin_interrupt_mask(dev, false);
	if (err != 0) {
		LOG_ERR("Failed to disable pin interrupt");
		goto error;
	}

	/* Clear all interrupt flags (discard the read value) */
	err = mrf24j40_read_interrupt_status_flags(dev, NULL);
	if (err != 0) {
		LOG_ERR("Failed to clear interrupt flags");
		goto error;
	}

	/* Disable the continuous carrier wave transmission (in case it was started) */
	if (config->pa_lna_ctrl_en) {
		testmode = MRF24J40_REG_TESTMODE_PA_LNA_MODE_VAL;
	}

	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_TESTMODE,
				  MRF24J40_REG_TESTMODE_TESTMODE_MASK, testmode);
	if (err != 0) {
		LOG_ERR("Failed to change the test mode back to its normal configuration value");
		goto error;
	}

	/* Reset RF state machine */
	err = mrf24j40_rf_state_machine_reset(dev);
	LOG_ERR("Failed to reset RF state machine");
	if (err != 0) {
		goto error;
	}

	err = mrf24j40_chip_interrupts_mask(dev, true);
	if (err != 0) {
		LOG_ERR("Failed to enable chip interrupts");
		goto error;
	}

	err = mrf24j40_mcu_pin_interrupt_mask(dev, true);
	if (err != 0) {
		LOG_ERR("Failed to enable pin interrupt");
		goto error;
	}

	k_mutex_unlock(&data->phy_mutex);

	LOG_DBG("MRF24J40 RF transceiver started");

	return err;

error:
	k_mutex_unlock(&data->phy_mutex);
	LOG_ERR("Error starting MRF24J40");
	return err;
}

static int mrf24j40_stop(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	struct mrf24j40_data *data = dev->data;
	uint8_t testmode = MRF24J40_REG_TESTMODE_NORMAL_MODE_VAL;
	int err = 0;

	k_mutex_lock(&data->phy_mutex, K_FOREVER);

	err = mrf24j40_chip_interrupts_mask(dev, false);
	if (err != 0) {
		LOG_ERR("Failed to mask chip interrupts");
		goto error;
	}

	err = mrf24j40_mcu_pin_interrupt_mask(dev, false);
	if (err != 0) {
		LOG_ERR("Failed to disable pin interrupt");
		goto error;
	}

	/* Disable the continuous carrier wave transmission (in case it was started) */
	if (config->pa_lna_ctrl_en) {
		testmode = MRF24J40_REG_TESTMODE_PA_LNA_MODE_VAL;
	}

	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_TESTMODE,
				  MRF24J40_REG_TESTMODE_TESTMODE_MASK, testmode);
	if (err != 0) {
		LOG_ERR("Failed to change the test mode back to its normal configuration value");
		goto error;
	}

	err = mrf24j40_rf_state_machine_reset(dev);
	if (err != 0) {
		LOG_ERR("Failed to reset RF state machine");
		goto error;
	}

	LOG_DBG("MRF24J40 RF transceiver stopped");
	k_mutex_unlock(&data->phy_mutex);

	return err;

error:
	k_mutex_unlock(&data->phy_mutex);
	LOG_ERR("Error stopping MRF24J40");
	return err;
}

static int mrf24j40_continuous_carrier(const struct device *dev)
{
	struct mrf24j40_data *data = dev->data;
	uint8_t testmode = MRF24J40_REG_TESTMODE_SINGLE_TONE_VAL;
	int err = 0;

	k_mutex_lock(&data->phy_mutex, K_FOREVER);

	/* Start transmitting the continuous carrier wave */
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_TESTMODE,
				  MRF24J40_REG_TESTMODE_TESTMODE_MASK, testmode);
	if (err != 0) {
		LOG_ERR("Could not start continuous carrier wave transmission");
	} else {
		LOG_DBG("Continuous carrier wave transmission started");
	}

	k_mutex_unlock(&data->phy_mutex);

	return err;
}

static uint16_t mrf24j40_get_subg_channel_count(const struct device *dev)
{
	ARG_UNUSED(dev);

	/* MRF24J40 does not support sub-GHz channels */
	return 0;
}

int mrf24j40_ed_scan(const struct device *dev, uint16_t duration, energy_scan_done_cb_t done_cb)
{
	int err = 0;
	uint8_t buf = 0;
	int16_t rssi_dbm;
	struct mrf24j40_data *data = dev->data;

	/*
	 * Note: MRF24J40 supports maximum ED duration of 8 symbol
	 * periods (8 * 16 us), so `duration` parameter is ignored.
	 */
	ARG_UNUSED(duration);

	k_mutex_lock(&data->phy_mutex, K_FOREVER);

	/* Start RSSI/ED mode 1 */
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_BBREG6, MRF24J40_REG_BBREG6_RSSIMODE1,
				  MRF24J40_REG_BBREG6_RSSIMODE1);

	/*
	 * Read RSSIRDY bit until it becomes 1 (ED cannot be done
	 * asynchronously - there is no interrupt for ED done on MRF24J40)
	 */
	do {
		err = mrf24j40_read(dev, MRF24J40_REG_ADDR_BBREG6, &buf, sizeof(buf));
	} while ((buf & MRF24J40_REG_BBREG6_RSSIRDY) == 0);

	/* Read detected RSSI */
	err = mrf24j40_read(dev, MRF24J40_REG_ADDR_RSSI, &buf, sizeof(buf));

	k_mutex_unlock(&data->phy_mutex);

	/* Convert measured raw RSSI value to RSSI in dBm */
	rssi_dbm = mrf24j40_rssi_raw_to_dbm(buf);

	/* Call ED done callback */
	if (done_cb) {
		done_cb(dev, rssi_dbm);
	}

	return err;
}

static int mrf24j40_beacon_nonbeacon_mode_set(const struct device *dev, bool nonbeacon_mode_en)
{
	int err = 0;
	uint8_t buf = 0;

	if (nonbeacon_mode_en) {
		/* Nonbeacon-enabled network mode */
		/* Clear SLOTTED bit */
		buf = 0;
		err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_TXMCR, MRF24J40_REG_TXMCR_SLOTTED,
					  buf);

		if (err == 0) {
			/* Configure beacon order and superframe order to 0x0F (no beacon) */
			buf = ((0x0Fu << MRF24J40_REG_ORDER_BO_SHIFT) &
			       MRF24J40_REG_ORDER_BO_MASK) |
			      (0x0Fu & MRF24J40_REG_ORDER_SO_MASK);
			err = mrf24j40_write(dev, MRF24J40_REG_ADDR_ORDER, &buf, sizeof(buf));
		}
	} else {
		/* Beacon-enabled network mode - not supported for now */
		err = -ENOTSUP;
	}

	return err;
}

static int mrf24j40_pan_coord_set(const struct device *dev, bool pan_coordinator_en)
{
	struct mrf24j40_data *data = dev->data;
	int err = 0;
	uint8_t buf = 0;

	if (pan_coordinator_en) {
		buf = MRF24J40_REG_RXMCR_PANCOORD;
	}

	k_mutex_lock(&data->phy_mutex, K_FOREVER);
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_RXMCR, MRF24J40_REG_RXMCR_PANCOORD, buf);
	k_mutex_unlock(&data->phy_mutex);

	return err;
}

static int mrf24j40_promiscuous_set(const struct device *dev, bool promiscuous_mode_en)
{
	struct mrf24j40_data *data = dev->data;
	int err = 0;
	uint8_t buf = 0;

	if (promiscuous_mode_en) {
		/* Enable promiscuous mode and disable automatic acknowledge responses */
		buf = MRF24J40_REG_RXMCR_PROMI | MRF24J40_REG_RXMCR_NOACKRSP;
	}

	k_mutex_lock(&data->phy_mutex, K_FOREVER);
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_RXMCR,
				  MRF24J40_REG_RXMCR_PROMI | MRF24J40_REG_RXMCR_NOACKRSP, buf);
	k_mutex_unlock(&data->phy_mutex);

	return err;
}

static int mrf24j40_configure(const struct device *dev, enum ieee802154_config_type type,
			      const struct ieee802154_config *config)
{
	int err = -EINVAL;

	switch (type) {
	case IEEE802154_CONFIG_PAN_COORDINATOR:
		err = mrf24j40_pan_coord_set(dev, config->pan_coordinator);
		break;

	case IEEE802154_CONFIG_PROMISCUOUS:
		err = mrf24j40_promiscuous_set(dev, config->promiscuous);
		break;

	default:
		break;
	}

	return err;
}

static int mrf24j40_interrupts_init(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	struct mrf24j40_data *data = dev->data;
	uint8_t buf;
	int err = 0;

	/* Configure interrupt polarity based on INT GPIO flags provided in device tree */
	if ((config->int_gpio.dt_flags & GPIO_ACTIVE_LOW) != 0) {
		/* Configure interrupt on falling edge */
		buf = ~((uint8_t)MRF24J40_REG_SLPCON0_INTEDGE);
	} else {
		/* Configure interrupt on rising edge */
		buf = MRF24J40_REG_SLPCON0_INTEDGE;
	}

	/* Write interrupt edge configuration to register */
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_SLPCON0, MRF24J40_REG_SLPCON0_INTEDGE,
				  buf);
	if (err != 0) {
		return err;
	}

	/* Register GPIO callback function and configure the GPIO peripheral */
	gpio_init_callback(&data->int_cb, mrf24j40_int_handler, BIT(config->int_gpio.pin));
	err = gpio_add_callback(config->int_gpio.port, &data->int_cb);
	if (err != 0) {
		return err;
	}

	/* Enable interrupt on MCU pin */
	err = mrf24j40_mcu_pin_interrupt_mask(dev, true);
	if (err != 0) {
		gpio_remove_callback(config->int_gpio.port, &data->int_cb);
	}

	/* Enable interrupts on chip: TXNIE, RXIE and SECIE */
	err = mrf24j40_chip_interrupts_mask(dev, true);
	if (err != 0) {
		return err;
	}

	return err;
}

static int mrf24j40_nonbeacon_init(const struct device *dev)
{
	uint8_t buf;
	int err = 0;

	/* Configure clear channel assessment mode to Energy above energy detection threshold */
	buf = 0x02u << MRF24J40_REG_BBREG2_CCAMODE_SHIFT;
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_BBREG2, MRF24J40_REG_BBREG2_CCAMODE_MASK,
				  buf);
	if (err != 0) {
		return err;
	}

	/* Set CCA energy detection threshold to recommended value (69 dBm) */
	buf = 0x60u;
	err = mrf24j40_write(dev, MRF24J40_REG_ADDR_CCAEDTH, &buf, sizeof(buf));
	if (err != 0) {
		return err;
	}

	/* Automatically calculate RSSI after each received packet and append it in RXFIFO */
	buf = 0x40;
	err = mrf24j40_write(dev, MRF24J40_REG_ADDR_BBREG6, &buf, sizeof(buf));
	if (err != 0) {
		return err;
	}

	err = mrf24j40_beacon_nonbeacon_mode_set(dev, false);

	return err;
}

static inline int mrf24j40_configure_mcu_gpios(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	int err = 0;

	/* Configure interrupt GPIO */
	if (!device_is_ready(config->int_gpio.port)) {
		return -ENODEV;
	}
	err = gpio_pin_configure_dt(&config->int_gpio, GPIO_INPUT);

	/* Configure reset GPIO (if it is provided in DT) */
	if (config->reset_gpio.port) {
		if (!device_is_ready(config->reset_gpio.port)) {
			return -EINVAL;
		}

		err = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
	}

	/* Configure wakeup GPIO (if it is provided in DT) */
	if (config->wake_gpio.port) {
		if (!device_is_ready(config->wake_gpio.port)) {
			return -EINVAL;
		}

		err = gpio_pin_configure_dt(&config->wake_gpio, GPIO_OUTPUT_ACTIVE);
	}

	return err;
}

static inline int mrf24j40_configure_transceiver_gpios(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	uint8_t testmode = MRF24J40_REG_TESTMODE_NORMAL_MODE_VAL;
	uint8_t gpio_dir = 0;
	uint8_t gpio_out = 0;
	int err = 0;

	if (config->pa_lna_ctrl_en) {
		/* Prepare GPIO0, GPIO1 and GPIO2 directions */
		gpio_dir |= (MRF24J40_REG_TRISGPIO_TRISGP0 | MRF24J40_REG_TRISGPIO_TRISGP1 |
			     MRF24J40_REG_TRISGPIO_TRISGP2);

		/*
		 * Change PA/LNA control configuration
		 * (GPIO0, GPIO1 and GPIO2 controlled by MRF24J40 state machine
		 */
		testmode = MRF24J40_REG_TESTMODE_PA_LNA_MODE_VAL;
	}

	if (config->gpio3_output != 0xFFu) {
		/* Prepare GPIO3 direction and output value */
		gpio_dir |= MRF24J40_REG_TRISGPIO_TRISGP3;
		if (config->gpio3_output == 1u) {
			gpio_out |= MRF24J40_REG_GPIO_GPIO3;
		}
	}

	if (config->gpio4_output != 0xFFu) {
		/* Prepare GPIO4 direction and output value */
		gpio_dir |= MRF24J40_REG_TRISGPIO_TRISGP4;
		if (config->gpio4_output == 1) {
			gpio_out |= MRF24J40_REG_GPIO_GPIO4;
		}
	}

	if (config->gpio5_output != 0xFFu) {
		/* Prepare GPIO5 direction and output value */
		gpio_dir |= MRF24J40_REG_TRISGPIO_TRISGP5;
		if (config->gpio5_output == 1) {
			gpio_out |= MRF24J40_REG_GPIO_GPIO5;
		}
	}

	/* Write GPIO pin directions */
	err = mrf24j40_write(dev, MRF24J40_REG_ADDR_TRISGPIO, &gpio_dir, sizeof(gpio_dir));
	if (err != 0) {
		return err;
	}

	/* Write GPIO pin output values */
	err = mrf24j40_write(dev, MRF24J40_REG_ADDR_GPIO, &gpio_out, sizeof(gpio_out));
	if (err != 0) {
		return err;
	}

	/* Write PA/LNA control configuration */
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_TESTMODE,
				  MRF24J40_REG_TESTMODE_TESTMODE_MASK, testmode);

	return err;
}

static int mrf24j40_chip_init(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	uint8_t buf;
	int err = 0;

	/* Do a software reset of the power, baseband and MAC circuitry */
	err = mrf24j40_software_reset(dev, true, true, true);
	if (err != 0) {
		return err;
	}

	/* Configure the chip's GPIO[0-5] pins according to properties from device tree  */
	err = mrf24j40_configure_transceiver_gpios(dev);
	if (err != 0) {
		return err;
	}

	/* Enable FIFO and set TXONTS to recommended value */
	buf = MRF24J40_REG_PACON2_FIFOEN | (0x06 << MRF24J40_REG_PACON2_TXONTS_SHIFT);
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_PACON2,
				  MRF24J40_REG_PACON2_FIFOEN | MRF24J40_REG_PACON2_TXONTS_MASK,
				  buf);
	if (err != 0) {
		return err;
	}

	/* Set RFSTBL to recommended value */
	buf = 0x09 << MRF24J40_REG_TXSTBL_RFSTBL_SHIFT;
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_TXSTBL, MRF24J40_REG_TXSTBL_RFSTBL_MASK,
				  buf);
	if (err != 0) {
		return err;
	}

	/* Set TURNTIME to recommended value */
	buf = (0x03 << MRF24J40_REG_TXTIME_TURNTIME_SHIFT) & MRF24J40_REG_TXTIME_TURNTIME_MASK;
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_TXTIME, MRF24J40_REG_TXTIME_TURNTIME_MASK,
				  buf);
	if (err != 0) {
		return err;
	}

	/* Set RFOPT to recommended value */
	buf = 0x03;
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_RFCON0, MRF24J40_REG_RFCON0_RFOPT_MASK,
				  buf);
	if (err != 0) {
		return err;
	}

	/* Set VCOOPT to recommended value */
	buf = 0x02;
	err = mrf24j40_write(dev, MRF24J40_REG_ADDR_RFCON1, &buf, sizeof(buf));
	if (err != 0) {
		return err;
	}

	/* Enable PLL */
	buf = MRF24J40_REG_RFCON2_PLLEN;
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_RFCON2, MRF24J40_REG_RFCON2_PLLEN, buf);
	if (err != 0) {
		return err;
	}

	/* Enable 20 MHz clock recovery and TX filter control */
	buf = MRF24J40_REG_RFCON6_20MRECVR | MRF24J40_REG_RFCON6_TXFIL;
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_RFCON6,
				  MRF24J40_REG_RFCON6_20MRECVR | MRF24J40_REG_RFCON6_TXFIL, buf);
	if (err != 0) {
		return err;
	}

	/* Sleep clock selection */
	if (config->sleep_clock_32khz_en) {
		/* Select external 32.768 kHz oscillator as sleep clock */
		buf = MRF24J40_REG_RFCON7_SLPCLKSEL_EXTERNAL << MRF24J40_REG_RFCON7_SLPCLKSEL_SHIFT;
		err = mrf24j40_write(dev, MRF24J40_REG_ADDR_RFCON7, &buf, sizeof(buf));
		if (err != 0) {
			return err;
		}
	} else {
		/* Select internal 100 kHz oscillator as sleep clock */
		buf = MRF24J40_REG_RFCON7_SLPCLKSEL_INTERNAL << MRF24J40_REG_RFCON7_SLPCLKSEL_SHIFT;
		err = mrf24j40_write(dev, MRF24J40_REG_ADDR_RFCON7, &buf, sizeof(buf));
		if (err != 0) {
			return err;
		}

		/* Disable CLK output and set sleep clock divider to 2 (=50 kHz sleep clock) */
		buf = MRF24J40_REG_SLPCON1_CLKOUTEN | (0x01 & MRF24J40_REG_SLPCON1_SLPCLKDIV_MASK);
		err = mrf24j40_write(dev, MRF24J40_REG_ADDR_SLPCON1, &buf, sizeof(buf));
		if (err != 0) {
			return err;
		}
	}

	/* Enable VCO */
	buf = MRF24J40_REG_RFCON8_RFVCO;
	err = mrf24j40_write(dev, MRF24J40_REG_ADDR_RFCON8, &buf, sizeof(buf));
	if (err != 0) {
		return err;
	}

	/* Flush RX FIFO */
	buf = MRF24J40_REG_RXFLUSH_RXFLUSH;
	err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_SLPCON0, MRF24J40_REG_RXFLUSH_RXFLUSH,
				  buf);
	if (err != 0) {
		return err;
	}

	if (config->nonbeacon_mode_en) {
		err = mrf24j40_nonbeacon_init(dev);
	} else {
		/* Slotted (beacon-enabled) mode not supported yet */
		err = -ENOTSUP;
	}
	if (err != 0) {
		return err;
	}

	err = mrf24j40_interrupts_init(dev);
	if (err != 0) {
		return err;
	}

	err = mrf24j40_set_txpower(dev, MRF24J40_DEFAULT_TX_POWER);
	if (err != 0) {
		return err;
	}

	err = mrf24j40_set_channel(dev, MRF24J40_DEFAULT_CHANNEL);
	if (err != 0) {
		return err;
	}

	if (config->turbo_mode_en) {
		err = mrf24j40_turbo_mode_enable(dev);
	}

	return err;
}

#ifdef CONFIG_PM_DEVICE
static int mrf24j40_pm_init(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	uint8_t buf;
	int err = 0;

	if (config->wake_gpio.port) {
		/*
		  Set WAKE pin to active on MCU before configuring
		  its polarity and enabling it on MRF24J40
		*/
		err = gpio_pin_set_dt(&config->wake_gpio, 1);
		if (err != 0) {
			return err;
		}

		if ((config->wake_gpio.dt_flags & GPIO_ACTIVE_LOW) != 0) {
			/* Configure WAKE pin on MRF24J40 to be active low */
			buf = ~((uint8_t)MRF24J40_REG_RXFLUSH_WAKEPOL);
		} else {
			/* Configure WAKE pin on MRF24J40 to be active high */
			buf = MRF24J40_REG_RXFLUSH_WAKEPOL;
		}

		/* Enable WAKE pin */
		buf |= MRF24J40_REG_RXFLUSH_WAKEPAD;
		err = mrf24j40_update_reg(
			dev, MRF24J40_REG_ADDR_RXFLUSH,
			MRF24J40_REG_RXFLUSH_WAKEPAD | MRF24J40_REG_RXFLUSH_WAKEPOL, buf);
		if (err != 0) {
			return err;
		}
	}

	/* Enable immediate wakeup mode */
	buf = MRF24J40_REG_WAKECON_IMMWAKE;
	return mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_WAKECON, MRF24J40_REG_WAKECON_IMMWAKE,
				   buf);
}
#endif /* CONFIG_PM_DEVICE */

static int mrf24j40_init(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	struct mrf24j40_data *data = dev->data;
	int err = 0;

	data->dev = dev;

	err = k_mutex_init(&data->phy_mutex);
	if (err != 0) {
		LOG_ERR("PHY mutex creation failed");
		return err;
	}

	err = k_sem_init(&data->isr_sem, 0, 1);
	if (err != 0) {
		LOG_ERR("ISR semaphore creation failed");
		return err;
	}

	err = k_sem_init(&data->tx_sem, 0, 1);
	if (err != 0) {
		LOG_ERR("TX semaphore creation failed");
		return err;
	}

	if (!spi_is_ready_dt(&config->bus)) {
		LOG_ERR("SPI bus not ready");
		return -EIO;
	}

	err = mrf24j40_configure_mcu_gpios(dev);
	if (err != 0) {
		LOG_ERR("Configuring MCU GPIOs (RST/INT/WAKE) failed");
		return err;
	}

	err = mrf24j40_hardware_reset(dev);
	if (err != 0) {
		LOG_ERR("Chip hardware reset failed");
		return err;
	}

	err = mrf24j40_chip_init(dev);
	if (err != 0) {
		LOG_ERR("Chip initialization failed");
		return err;
	}

#ifdef CONFIG_PM_DEVICE
	err = mrf24j40_pm_init(dev);
	if (err != 0) {
		LOG_ERR("Chip power management initialization failed");
		return err;
	}
#endif /* CONFIG_PM_DEVICE */

	/* Create a thread for processing packets */
	k_thread_create(&data->mrf24j40_thread, data->mrf24j40_thread_stack,
			CONFIG_IEEE802154_MRF24J40_THREAD_STACK_SIZE,
			(k_thread_entry_t)mrf24j40_thread_main, (void *)dev, NULL, NULL,
			K_PRIO_COOP(2), 0, K_NO_WAIT);
	k_thread_name_set(&data->mrf24j40_thread, "mrf24j40_thread");

	return err;
}

static void mrf24j40_iface_init(struct net_if *iface)
{
	const struct device *dev = net_if_get_device(iface);
	struct mrf24j40_data *data = dev->data;

	/* If not specified in device tree, fill data->mac_addr with random numbers */
	mrf24j40_get_mac(dev);

	net_if_set_link_addr(iface, data->mac_addr, 8, NET_LINK_IEEE802154);

	data->iface = iface;

	ieee802154_init(iface);

	LOG_DBG("MRF24J40 net interface initialized.");
}

#ifdef CONFIG_PM_DEVICE
static int mrf24j40_suspend(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	int err = 0;

	/* If WAKE pin is provided in device tree, set it to inactive state */
	if (config->wake_gpio.port) {
		err = gpio_pin_set_dt(&config->wake_gpio, 0);
	}

	if (err == 0) {
		/* Reset power management circuitry */
		err = mrf24j40_software_reset(dev, false, false, true);
		if (err == 0) {
			/* Put device to sleep by writing to SLPACK register */
			err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_SLPACK,
						  MRF24J40_REG_SLPACK_SLPACK,
						  MRF24J40_REG_SLPACK_SLPACK);
		}
	}

	return err;
}

static int mrf24j40_resume(const struct device *dev)
{
	const struct mrf24j40_config *config = dev->config;
	int err = 0;

	if (config->wake_gpio.port) {
		/* WAKE pin provided in device tree, wake up by setting it to active state */
		err = gpio_pin_set_dt(&config->wake_gpio, 1);
	} else {
		/* WAKE pin not provided in device tree, wake up by writing to WAKECON register */
		/* Set REGWAKE bit */
		err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_WAKECON,
					  MRF24J40_REG_WAKECON_REGWAKE,
					  MRF24J40_REG_WAKECON_REGWAKE);
		if (err == 0) {
			/* Clear REGWAKE bit */
			err = mrf24j40_update_reg(dev, MRF24J40_REG_ADDR_WAKECON,
						  MRF24J40_REG_WAKECON_REGWAKE,
						  ~((uint8_t)MRF24J40_REG_WAKECON_REGWAKE));
		}
	}

	if (err == 0) {
		/* Reset RF state machine after wakeup */
		err = mrf24j40_rf_state_machine_reset(dev);
		if (err == 0) {
			/* Delay for 2 ms to allow 20 MHz oscillator to stabilize */
			k_msleep(2);
		}
	}

	return err;
}

static int mrf24j40_pm_action(const struct device *dev, enum pm_device_action action)
{
	int err = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		err = mrf24j40_resume(dev);
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		err = mrf24j40_suspend(dev);
		break;
	default:
		err = -ENOTSUP;
		break;
	}

	if (err < 0) {
		LOG_ERR("%s: failed to set power mode %d", dev->name, action);
	}

	return err;
}
#endif /* CONFIG_PM_DEVICE */

static struct ieee802154_radio_api mrf24j40_radio_api = {
	.iface_api.init = mrf24j40_iface_init,
	.get_capabilities = mrf24j40_get_capabilities,
	.cca = mrf24j40_cca,
	.set_channel = mrf24j40_set_channel,
	.filter = mrf24j40_filter,
	.set_txpower = mrf24j40_set_txpower,
	.start = mrf24j40_start,
	.stop = mrf24j40_stop,
	.tx = mrf24j40_tx,
	.continuous_carrier = mrf24j40_continuous_carrier,
	.configure = mrf24j40_configure,
	.get_subg_channel_count = mrf24j40_get_subg_channel_count,
	.ed_scan = mrf24j40_ed_scan,
};

#define IEEE802154_MRF24J40_RAW_DEVICE_INIT(inst)                                                  \
	DEVICE_DT_INST_DEFINE(inst, mrf24j40_init, PM_DEVICE_DT_INST_GET(inst),                    \
			      &mrf24j40_data_##inst, &mrf24j40_config_##inst, POST_KERNEL,         \
			      CONFIG_IEEE802154_MRF24J40_INIT_PRIO, &mrf24j40_radio_api);

#define IEEE802154_MRF24J40_NET_DEVICE_INIT(inst)                                                  \
	NET_DEVICE_DT_INST_DEFINE(inst, mrf24j40_init, PM_DEVICE_DT_INST_GET(inst),                \
				  &mrf24j40_data_##inst, &mrf24j40_config_##inst,                  \
				  CONFIG_IEEE802154_MRF24J40_INIT_PRIO, &mrf24j40_radio_api,       \
				  IEEE802154_L2, NET_L2_GET_CTX_TYPE(IEEE802154_L2),               \
				  MRF24J40_PSDU_LENGTH);

#define IEEE802154_MRF24J40_INIT(inst)                                                             \
                                                                                                   \
	PM_DEVICE_DT_INST_DEFINE(inst, mrf24j40_pm_action);                                        \
                                                                                                   \
	static const struct mrf24j40_config mrf24j40_config_##inst = {                             \
		.bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0),          \
		.reset_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, reset_gpios, {0}),                    \
		.int_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, int_gpios, {0}),                        \
		.wake_gpio = GPIO_DT_SPEC_INST_GET_OR(inst, wake_gpios, {0}),                      \
		.has_mac = DT_INST_NODE_HAS_PROP(inst, mac_addr),                                  \
		.nonbeacon_mode_en = DT_INST_PROP(inst, nonbeacon_mode_en),                        \
		.turbo_mode_en = DT_INST_PROP(inst, turbo_mode_en),                                \
		.sleep_clock_32khz_en = DT_INST_PROP(inst, sleep_clock_32khz_en),                  \
		.pa_lna_ctrl_en = DT_INST_PROP(inst, pa_lna_ctrl_en),                              \
		.gpio3_output = DT_INST_PROP(inst, gpio3_output),                                  \
		.gpio4_output = DT_INST_PROP(inst, gpio4_output),                                  \
		.gpio5_output = DT_INST_PROP(inst, gpio5_output),                                  \
	};                                                                                         \
                                                                                                   \
	static struct mrf24j40_data mrf24j40_data_##inst = {                                       \
		.mac_addr = UTIL_AND(DT_INST_NODE_HAS_PROP(inst, mac_addr),                        \
				     UTIL_AND(DT_INST_PROP_LEN(inst, mac_addr) == 8,               \
					      DT_INST_PROP(inst, mac_addr)))};                     \
                                                                                                   \
	COND_CODE_1(CONFIG_IEEE802154_RAW_MODE, (IEEE802154_MRF24J40_RAW_DEVICE_INIT(inst)),       \
		    (IEEE802154_MRF24J40_NET_DEVICE_INIT(inst)))

DT_INST_FOREACH_STATUS_OKAY(IEEE802154_MRF24J40_INIT)
