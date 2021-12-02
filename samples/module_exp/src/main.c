/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <string.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>

#include <stdlib.h>
#include <arm_math.h>
#include <arm_const_structs.h>

#define FFT_SAMPLES 1024
#define FFT_SAMPLES_HALF (FFT_SAMPLES / 2)

static float32_t complexFFT[FFT_SAMPLES], realFFT[FFT_SAMPLES_HALF],
		imagFFT[FFT_SAMPLES_HALF], angleFFT[FFT_SAMPLES_HALF],
		powerFFT[FFT_SAMPLES_HALF];
		
void main(void)
{
	const struct device *dev = device_get_binding(
			CONFIG_UART_CONSOLE_ON_DEV_NAME);
	uint32_t dtr = 0;

	if (usb_enable(NULL)) {
		return;
	}

	/* Poll if the DTR flag was set, optional */
	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
	}

	if (strlen(CONFIG_UART_CONSOLE_ON_DEV_NAME) !=
	    strlen("CDC_ACM_0") ||
	    strncmp(CONFIG_UART_CONSOLE_ON_DEV_NAME, "CDC_ACM_0",
		    strlen(CONFIG_UART_CONSOLE_ON_DEV_NAME))) {
		printk("Error: Console device name is not USB ACM\n");

		return;
	}
	while(1){
		printk("Hello\n");
	}
}
