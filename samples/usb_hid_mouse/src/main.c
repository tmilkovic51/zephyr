/*
 * Copyright (c) 2018 qianfan Zhao
 * Copyright (c) 2018, 2023 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/util.h>

#include <zephyr/usb/usb_device.h>
#include <zephyr/usb/class/usb_hid.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

static const uint8_t hid_report_desc[] = HID_MOUSE_REPORT_DESC(2);

#define MOUSE_MOVE_AMOUNT	20

enum mouse_report_idx {
	MOUSE_BTN_REPORT_IDX = 0,
	MOUSE_X_REPORT_IDX = 1,
	MOUSE_Y_REPORT_IDX = 2,
	MOUSE_WHEEL_REPORT_IDX = 3,
	MOUSE_REPORT_COUNT = 4,
};

static const struct device *hid_dev;
static uint8_t report[MOUSE_REPORT_COUNT];

static void move_mouse(int x, int y)
{
	int ret;
	report[MOUSE_X_REPORT_IDX] = x;
	report[MOUSE_Y_REPORT_IDX] = y;

	ret = hid_int_ep_write(hid_dev, report, sizeof(report), NULL);
	if (ret) {
		LOG_ERR("HID write error, %d", ret);
	}
}

int main(void)
{
	int ret;

	hid_dev = device_get_binding("HID_0");
	if (hid_dev == NULL) {
		LOG_ERR("Cannot get USB HID Device");
		return 0;
	}

	usb_hid_register_device(hid_dev,
				hid_report_desc, sizeof(hid_report_desc),
				NULL);

	usb_hid_init(hid_dev);

	ret = usb_enable(NULL);
	if (ret != 0) {
		LOG_ERR("Failed to enable USB");
		return 0;
	}

	while (true) {
		move_mouse(MOUSE_MOVE_AMOUNT, 0);
		k_msleep(1000);
		move_mouse(0, 0);
		k_msleep(1000);
		move_mouse(0, MOUSE_MOVE_AMOUNT);
		k_msleep(1000);
		move_mouse(0, 0);
		k_msleep(1000);
		move_mouse(-MOUSE_MOVE_AMOUNT, 0);
		k_msleep(1000);
		move_mouse(0, 0);
		k_msleep(1000);
		move_mouse(0, -MOUSE_MOVE_AMOUNT);
		k_msleep(1000);
		move_mouse(0, 0);
		k_msleep(1000);
	}
	return 0;
}
