/*
 * Copyright (c) 2016 Intel Corporation
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file Sample app to demonstrate PWM-based LED fade
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER (main);

static const struct device *mrf24j40_device = DEVICE_DT_GET(DT_CHOSEN(zephyr_ieee802154));

int main(void)
{
	int ret = 0;

	LOG_INF("MRF24J40 demo start");

	if (!device_is_ready(mrf24j40_device)) {
		LOG_ERR("Error: MRF24j40 device %s is not ready",
		       mrf24j40_device->name);
		return 0;
	}

	while (1) {

		// Sleep for 2 seconds
		k_sleep(K_MSEC(2000));
	}

	return ret;
}
