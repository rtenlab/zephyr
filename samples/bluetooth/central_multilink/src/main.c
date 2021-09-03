/* main.c - Application main entry point */

/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

int init_central(void);

#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>

void main(void)
{const struct device *dev_led, *dev_usb;
        int err;


        /* USB serial setup */
  dev_usb = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);
        uint32_t dtr = 0;

        if (usb_enable(NULL)) {
                return;
        }

        while (!dtr) {
                uart_line_ctrl_get(dev_usb, UART_LINE_CTRL_DTR, &dtr);
        }

        /* Initialize the Bluetooth Subsystem */
        printk("Starting Beacon Demo\n");
	printk("This is the first changes made by Dev on this platform\n");
	(void)init_central();
}
