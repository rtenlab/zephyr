/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)

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

/* The devicetree node identifier for the "led0" alias. */
#define LED1_NODE DT_ALIAS(led1)

#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
#define LED1	DT_GPIO_LABEL(LED1_NODE, gpios)
#define LED1_PIN	DT_GPIO_PIN(LED1_NODE, gpios)
#define LED1_FLAGS	DT_GPIO_FLAGS(LED1_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define PIN	0
#define FLAGS	0
#endif


/* The devicetree node identifier for the "led0" alias. */
#define TOGGLE_NODE DT_ALIAS(toggle0)

#if DT_NODE_HAS_STATUS(TOGGLE_NODE, okay)
#define TOGGLE	DT_GPIO_LABEL(TOGGLE_NODE, gpios)
#define TOGGLE_PIN	DT_GPIO_PIN(TOGGLE_NODE, gpios)
#define TOGGLE_FLAGS	DT_GPIO_FLAGS(TOGGLE_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define TOGGLE	""
#define TOGGLE_PIN	0
#define TOGGLE_FLAGS	0
#endif



void led_on_blink0(bool flag)
{
	const struct device *dev;
	int ret;

	dev = device_get_binding(LED0);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, PIN, GPIO_OUTPUT_ACTIVE | FLAGS);
	if (ret < 0) {
		return;
	}

	gpio_pin_set(dev, PIN, (int)flag);
}


void led_on_blink1(bool flag)
{
	const struct device *dev;
	int ret;

	dev = device_get_binding(LED1);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, LED1_PIN, GPIO_OUTPUT_ACTIVE | LED1_FLAGS);
	if (ret < 0) {
		return;
	}

	gpio_pin_set(dev, LED1_PIN, (int)flag);
}

void toggle_pin_D9(bool flag)
{
	const struct device *dev;
	int ret;

	dev = device_get_binding(TOGGLE);
	if (dev == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev, TOGGLE_PIN, GPIO_OUTPUT_ACTIVE | TOGGLE_FLAGS);
	if (ret < 0) {
		return;
	}

	gpio_pin_set(dev, TOGGLE_PIN, (int)flag);
}