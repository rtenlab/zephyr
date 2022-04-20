/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uart_i2c.h"
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <stdio.h>
#include <drivers/gpio.h>
#include "sht31.h"
#include "Onewire.h"
#include "Dallas_temperature.h"

extern const struct device *dev_ds18b20;

void main(void)
{
	
	int ret;
	enable_uart_console();
	configure_device();

	dev_ds18b20 = device_get_binding(LED1);
	if (dev_ds18b20 == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev_ds18b20, LED1_PIN, GPIO_OUTPUT);
	if (ret < 0) {
		return;
	}

	{
		volatile int n_devices;
		sht31_t sht31_data;
		DallasTemperature_begin();		

		n_devices = getDeviceCount();

		while (1) {
			read_temp_hum(&sht31_data);
			print_data_sht(&sht31_data);
			requestTemperatures();
			printf("Total devices: %d\n", n_devices);
			for (int i = 0; i < n_devices; i++) {
				printf("sensor %d: %f\n", i, getTempCByIndex(i));
			}
			k_msleep(1000);
		}
	}
}
