/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include "uart_i2c.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led2)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define PIN	DT_GPIO_PIN(LED0_NODE, gpios)
#define FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif



void main(void)
{
	// enable_uart_console();
	const struct device *dev;
	bool led_is_on = true;
	int ret;

	dev = device_get_binding(LED0);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE);
	if (ret < 0) {
		return;
	}
	while (1) {
		gpio_pin_configure(dev, PIN, GPIO_OUTPUT);
		gpio_pin_set(dev, PIN, (int)true);
		k_busy_wait(480);
		gpio_pin_set(dev,PIN,(int)false);
		k_busy_wait(1);		
		gpio_pin_configure(dev, PIN, GPIO_INPUT);
		int r = gpio_pin_get(dev, PIN);	
		// printf("%d\n",r);
		k_busy_wait(10);
	}
}
