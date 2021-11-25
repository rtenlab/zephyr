/*
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * Adafruit LSM6DS Library: https://github.com/adafruit/Adafruit_LSM6DS
 * Adafruit I2C support library: https://github.com/adafruit/Adafruit_BusIO
 * Zephyr I2C documentation: https://docs.zephyrproject.org/latest/reference/peripherals/i2c.html
 */




/**
 * @file: This app enables the use of LSM6DS33 sensor on zephyr OS.
 */
#include "lsm6ds33.h"



void main(void){
	lsm6ds33_t all_data;
	// enabling uart_console for zephyr.
	enable_uart_console();

	// configuring the i2c device for communication with the sensor.
	configure_device();

	// check if we have the sensor we want.
	check_device();

	// Initialize the sensor by setting necessary parameters for gyroscope and accelerometer
	lsm6ds33_init();

	
	

	while(1){
		delay(90);
		read_burst_data(&all_data);
		print_data(&all_data);
	}

		
	return;
}
