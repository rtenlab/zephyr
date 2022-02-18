/*
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * Adafruit SHT3X Library: https://github.com/adafruit/Adafruit_SHT31/blob/master/Adafruit_SHT31.cpp
 * Adafruit I2C support library: https://github.com/adafruit/Adafruit_BusIO
 * Zephyr I2C documentation: https://docs.zephyrproject.org/latest/reference/peripherals/i2c.html
 */




/**
 * @file: This app enables the use if SHT-31 sensor on zephyr OS.
 */

#include "sht31.h"
#include "uart_i2c.h"

void main(void){
	sht31_t variable;
	enable_uart_console();
	configure_device();
	uint8_t temp[2];
	while(true){
		read_temp_hum(&variable);
		print_data_sht(&variable);
		temp[0] = (uint8_t)variable.temp;
		temp[1] = (uint8_t)((variable.temp-temp[0])*100);
		printk("UINT8_T TEMP[0]: %d\n", temp[0]);
		printk("UINT8_T TEMP[1]: %d\n", temp[1]);
		delay(10);
	}
	return;
}
