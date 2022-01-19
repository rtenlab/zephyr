/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uart_i2c.h"
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>

#include <drivers/gpio.h>


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led1)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_PIN		DT_GPIO_PIN(LED0_NODE, gpios)
#define LED0_FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define LED0_PIN	0
#define LED0_FLAGS	0
#endif

#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
#define LED1	DT_GPIO_LABEL(LED1_NODE, gpios)
#define LED1_PIN		DT_GPIO_PIN(LED1_NODE, gpios)
#define LED1_FLAGS	DT_GPIO_FLAGS(LED1_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED1	""
#define LED1_PIN	0
#define LED1_FLAGS	0
#endif

void main(void)
{
	const struct device *dev0, *dev1;
	bool led_is_on = true;
	int ret;

	dev0 = device_get_binding(LED0);
	if (dev0 == NULL) {
		return;
	}
	dev1 = device_get_binding(LED1);
	if (dev1 == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev0, LED0_PIN, GPIO_OUTPUT_ACTIVE | LED0_FLAGS);
	if (ret < 0) {
		return;
	}
	ret = gpio_pin_configure(dev1, LED1_PIN, GPIO_OUTPUT_ACTIVE | LED1_FLAGS);
	if (ret < 0) {
		return;
	}

	while (1) {
		gpio_pin_set(dev0, LED0_PIN, (int)led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME_MS);
		gpio_pin_set(dev0, LED0_PIN, (int)led_is_on);
		k_msleep(SLEEP_TIME_MS);
		gpio_pin_set(dev1, LED1_PIN, (int)!led_is_on);
		led_is_on = !led_is_on;
		k_msleep(SLEEP_TIME_MS);
		gpio_pin_set(dev1, LED1_PIN, (int)!led_is_on);
		k_msleep(SLEEP_TIME_MS);
	}
}
