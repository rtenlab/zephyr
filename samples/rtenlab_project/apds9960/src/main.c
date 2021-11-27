/*
 * Copyright (c) 2018 Tavish Naruka <tavishnaruka@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * More data_proximity about the apds9960 sensor registers can be find on the datasheet: https://cdn-learn.adafruit.com/assets/assets/000/045/848/original/Avago-APDS-9960-datasheet.pdf?1504034182
 */



#include "uart_i2c.h"
#include "apds9960.h"
/**
 * @file This app enables the proximity bit on the apds9960 sensor and reads the value from a register specified for reading the proximity data_proximity. It also establishes a usb console for reading the printk statements.
 */

void main(void){
	apds9960_t apds_sensor_data;
	// enable uart console to send data over uart console.
	enable_uart_console();
	// This is to configure i2c device.
	configure_device();
	// Enablling apds sensor by setting particular bits.
	enable_apds_sensor();
	
	while(1){
		// read proximity data from the sensor.
		read_proximity_data(&apds_sensor_data);
		// read ALS data from the sensor.
		read_als_data(&apds_sensor_data);
		// Print the data captured until now.
    	print_data_apds(&apds_sensor_data);
		//delay for 100 msecs.
		delay(100);
	}
    
  }
