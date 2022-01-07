/*
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * Reference library: https://github.com/Sensirion/embedded-i2c-scd4x
 * Adafruit I2C support library: https://github.com/adafruit/Adafruit_BusIO
 * Zephyr I2C documentation: https://docs.zephyrproject.org/latest/reference/peripherals/i2c.html
 */


/** 
 * Cable configuration:
 * Green-cable: SDA
 * Red cable: VDD
 * Yellow cable: SCL
 * Black cable: GND
 * I2C ADDRESS FOR THE SENSOR IS 0X62
 */

/**
 * @file: This app enables the use of SCD41 CO2 Sensor  sensor on zephyr OS.
 */

#include "scd41.h"


void main(void){
	//struct to store the data measured from the sensor.
	scd41_t sensor_data;

	// enabling uart_console for zephyr.
	enable_uart_console();

	// configuring the i2c device for communication with the sensor.
	configure_device();

	//print the serial id of the sensor.
	get_serial_number();

	// Infinite loop to measure the data from the sensor, every 100ms. Overall latency will be 5103 ms for each loop iteration.
	while(1){
	measure_single_shot(&sensor_data);
	print_data_scd(&sensor_data);
	delay(100);
	}

	return;
}
