/*
 * Copyright (c) 2018 Tavish Naruka <tavishnaruka@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * More data_proximity about the apds9960 sensor registers can be find on the datasheet: https://cdn-learn.adafruit.com/assets/assets/000/045/848/original/Avago-APDS-9960-datasheet.pdf?1504034182
 * I2C Interface for Adafruit: https://github.com/adafruit/Adafruit_BusIO/blob/master/Adafruit_I2CDevice.cpp
 * BMP280 Library: https://github.com/adafruit/Adafruit_BMP280_Library
 *
 */





/**
 * @file This app enables the proximity bit on the apds9960 sensor and reads the value from a register specified for reading the proximity data_proximity. It also establishes a usb console for reading the printk statements.
 */

#include "bmp280.h"
																			// Macro for masking bits.
#define DEBUG 0
const struct device *dev_i2c;															// Device struct to get device binding for I2C



/**
Dig_T1 = calib_data_unsigned_registers[0]				0x88/0x89
Dig_T2 = calib_data_signed_registers[0]					0x8A/0x8B
Dig_T3 = calib_data_signed_registers[1]					0x8C/0x8D
Dig_P1 = calib_data_unsigned_registers[1]				0x8E/0x8F
Dig_P2 = calib_data_signed_registers[2]					0x90/0x91
Dig_P3 = calib_data_signed_registers[3]					0x92/0x93
Dig_P4 = calib_data_signed_registers[4]					0x94/0x95
Dig_P5 = calib_data_signed_registers[5]					0x96/0x97
Dig_P6 = calib_data_signed_registers[6]					0x98/0x99
Dig_P7 = calib_data_signed_registers[7]					0x9A/0x9B
Dig_P8 = calib_data_signed_registers[8]					0x9C/0x9D
Dig_P9 = calib_data_signed_registers[9]					0x9E/0x9F
*/






/**
* FUNCTION: Main function!
*/

void main(void){
	bmp280_t bmp280_sensor_data;
	enable_uart_console();
	configure_device();
	read_calibration_registers();
   
	if(check_sensor()){
		printf("Problem with the code returning...");
	}
	
	while(1){
	delay(500);
	bmp_read_press_temp_data(&bmp280_sensor_data);
	print_data_bmp(&bmp280_sensor_data);
	}
  return;
}
