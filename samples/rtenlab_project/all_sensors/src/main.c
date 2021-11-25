/*
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * Reference library: https://github.com/Sensirion/embedded-i2c-scd4x
 * Adafruit I2C support library: https://github.com/adafruit/Adafruit_BusIO
 * Zephyr I2C documentation: https://docs.zephyrproject.org/latest/reference/peripherals/i2c.html
 */




/**
 * @file: This app intergrates all the sensors of Adafruit_feather_sense board on zephyr OS.
 */

#include "scd41.h"
#include "lsm6ds33.h"
#include "scd41.h"

void main(void){
	//struct to store the data measured from the sensor.

	// enabling uart_console for zephyr.
	enable_uart_console();

	// configuring the i2c device for communication with the sensor.
	configure_device();

	get_serial_number();
	check_device();

	// Infinite loop to measure the data from the sensor, every 100ms. Overall latency will be 5103 ms for each loop iteration.
	while(1){
	delay(100);
	}

	return;
}
