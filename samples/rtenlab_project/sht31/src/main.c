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

void main(void){
	temphum_t variable;
	enable_uart_console();
	configure_device();
	
	while(true){
		read_temp_hum(&variable);
		print_data(&variable);
		delay(10);
	}
	return;
}
