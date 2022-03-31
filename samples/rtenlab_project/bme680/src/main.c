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

	bme680_gas_par_t gascalib;
	bme680_gas_data_t gasdata;
	if(!bme680_get_gas_calib_data(&gascalib)){
		printk("BME680: Gas Calib data returned with false at line %d\n", __LINE__);
	}
#ifdef DEBUG
	printk("%d\n",gascalib.para1);
	printk("%d\n",gascalib.para2);
	printk("%d\n",gascalib.para3);
	printk("%d\n",gascalib.res_heat_range);
	printk("%d\n",gascalib.res_heat_val);
	printk("%d\n",gascalib.range_sw_err);
#endif

	while(1){
		bme680_set_heater_conf(320,150,&gascalib);
		bme680_set_power_mode(1);
		#ifdef DEBUG
		printk("Before: ");
		bme680_check_new_data();
		#endif
		delay(200);
		#ifdef DEBUG
		printk("After: ");
		bme680_check_new_data();
		#endif
		bme680_get_raw_gas_data(&gasdata);
		printk("Value of the gas sensor output is: %f KOhms\n",bme680_calc_gas_resistance(&gasdata, &gascalib)/1000.00);
	}

	return;
}
