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





//#define FIRE_SCD
#define FIRE_SHT
#define FIRE_LSM
#define FIRE_APDS

#ifdef FIRE_APDS
#include "apds9960.h"
void apds9960_fire(apds9960_t* apds9960_data){
	read_proximity_data(apds9960_data);
	read_als_data(apds9960_data);
	print_data_apds(apds9960_data);
	return;
}
#endif

#ifdef FIRE_SCD
#include "scd41.h"
void scd41_fire(scd41_t* scd41_data){
	measure_single_shot(scd41_data);
	printk("Printing SCD41 DATA!!!\n");
	print_data_scd(scd41_data);

}
#endif

#ifdef FIRE_SHT
#include "sht31.h"
void sht31_fire(sht31_t* sht31_data){
	read_temp_hum(sht31_data);
	printk("PRINTING SHT31 DATA!!!\n");
	print_data_sht(sht31_data);
}
#endif

#ifdef FIRE_LSM
#include "lsm6ds33.h"
void lsm6ds33_fire(lsm6ds33_t* lsm6ds33_data){
	read_burst_data(lsm6ds33_data);
	printk("PRINTING LSM6DS33 DATA!!!\n");
	print_data_lsm(lsm6ds33_data);
}
#endif
void main(void){
	// enabling uart_console for zephyr.
	enable_uart_console();

	// configuring the i2c device for communication with the sensor.
	configure_device();

	//struct to store the data measured from the sensor.
	#ifdef FIRE_SCD
	scd41_t scd41_sensor_data;
	get_serial_number();
	#endif

	#ifdef FIRE_SHT
	sht31_t sht31_sensor_data;
	#endif

	#ifdef FIRE_LSM
	lsm6ds33_t lsm6ds33_sensor_data;
	lsm6ds33_init();
	#endif
	
	#ifdef FIRE_APDS
	apds9960_t apds9960_sensor_data;
	enable_apds_sensor();
	#endif

	
	
	// Infinite loop to measure the data from the sensor, every 100ms. Overall latency will be 5103 ms for each loop iteration.
	while(1){
	#ifdef FIRE_SCD
		scd41_fire(&scd41_sensor_data);
		delay(100);
	#endif
	#ifdef FIRE_SHT
		sht31_fire(&sht31_sensor_data);
		delay(50);
	#endif
	#ifdef FIRE_LSM
		lsm6ds33_fire(&lsm6ds33_sensor_data);
		delay(50);
	#endif
	#ifdef FIRE_APDS
		apds9960_fire(&apds9960_sensor_data);
		delay(50);
	#endif
	}

	return;
}
