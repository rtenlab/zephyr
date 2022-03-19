/*
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * Reference library: https://github.com/BoschSensortec/BSEC-Arduino-library/tree/master/src/bme680
 * Adafruit I2C support library: https://github.com/adafruit/Adafruit_BusIO
 * Zephyr I2C documentation: https://docs.zephyrproject.org/latest/reference/peripherals/i2c.html
 */


/** 
 * Pin configuration:
 * (Sensor Pin) -> (Board Pin)
 * 3Vo -> 3V
 * GND -> GND
 * 
 * I2C ADDRESS FOR THE SENSOR IS 0X62
 */

/**
 * @file: This app enables the use of SCD41 CO2 Sensor  sensor on zephyr OS.
 */

#include "bme680.h"
#include "uart_i2c.h"


void main(void){
	// enabling uart_console for zephyr.
	enable_uart_console();

	// configuring the i2c device for communication with the sensor.
	configure_device();

	if(!bme680_getid()){
		printk("BME680: Get ID returned with false at line %d\n", __LINE__);
	}
	return;
}
