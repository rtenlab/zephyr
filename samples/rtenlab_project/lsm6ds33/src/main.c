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

// #define SINGLE_SHOT_MODE
#define FIFO_MODE

void main(void){
	lsm6ds33_t lsm6ds33_sensor_data;
	// enabling uart_console for zephyr.
	enable_uart_console();

	// configuring the i2c device for communication with the sensor.
	configure_device();

	// check if we have t he sensor we want.
	check_device();

	// Initialize the sensor by setting necessary parameters for gyroscope and accelerometer
	lsm6ds33_init();

	accel_set_power_mode(ACCEL_LOW_POWER_MODE);
	gyro_set_power_mode(GYRO_LOW_POWER_MODE);
	
	
	
#ifdef SINGLE_SHOT_MODE  
	while(1){
		// currently working on 26Hz frequency. Set the delay accordingly.
		delay(1);
		status_reg();
		get_accel_rate();
		read_burst_data(&lsm6ds33_sensor_data);
		print_data(&lsm6ds33_sensor_data);
	}
#endif

#ifdef FIFO_MODE
/**
 * @brief  Using the FIFO mode. Still some things needs to be done! 
 * @note   Check for the decimation rate and the ODR.
 * @retval 
 */

	while(1){
		printk("Waiting for watermark...\nIf you want to see some changes in the readings change the orientation of the board right now!!!\n");

		//  Check if the FIFO_FULL Flag is set or not.
		while((lsm6ds33_fifo_status() & FIFO_FULL) == 0) { };

		while( (lsm6ds33_fifo_status()& FIFO_EMPTY) == 0){
			printk("AccelX_Raw: %f\n", lsm6ds33_fifo_get_accel_data(lsm6ds33_fifo_read()));
			printk("AccelY_Raw: %f\n", lsm6ds33_fifo_get_accel_data(lsm6ds33_fifo_read()));
			printk("AccelZ_Raw: %f\n", lsm6ds33_fifo_get_accel_data(lsm6ds33_fifo_read()));
			delay(50);
		}
		lsm6ds33_fifo_change_mode(0);

		lsm6ds33_fifo_change_mode(1);

	}
#endif
		
	return;
}
