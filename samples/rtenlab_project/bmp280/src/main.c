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

#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>


/**
 * @file This app enables the proximity bit on the apds9960 sensor and reads the value from a register specified for reading the proximity data_proximity. It also establishes a usb console for reading the printk statements.
 */


#define I2C_DEV "I2C_0"
#define mask(x) (1<<x)																				// Macro for masking bits.
#define DEBUG 0

uint8_t data_bmp280=1;
const struct device *dev_bmp280;															// Device struct to get device binding for I2C
uint8_t BMP280_ADDR = 0x77;																		// Address of the Sensor on I2C bus as described by Adafruit website
uint8_t temp_data[3];																					// Array to hold the raw temperature data.
uint8_t press_data[3];																				// Array to hold the raw pressure data.
uint32_t temperature, pressure;																// 32 bit variable to convert the 20 bit data from the array of registers to a single variable
uint8_t calib_data[2];																				// dump array of registers to burst read the 16 bit calib data.
uint16_t calib_data_unsigned_registers[2];										//Contains dig_T1 and dig_P1 register.
int16_t calib_data_signed_registers[10];											// Containes dig_T2,T3 and dig_[P2-P9] registers.
uint8_t calib_signed_registers[10]= {0x8A,0x8C,0x90,0x92,
						0x94,0x96,0x98,0x9A,0x9C,0x9E};										// Register address to get the calibaration data.
uint8_t calib_unsigned_registers[2] = {0x88,0x8E};						// Register address to get the unsigned calibration data.
int32_t t_fine;

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
*	FUNCTION: To read the calibration registers required for compensation of the raw temperature and pressure data.
*/

void read_calibration_registers(void){
	printk("I\t calib[0]\t calib[1]\t final_temperature\t Register\n");

	// For loop tp get the unsigned data dig_T1 and dig_P1.
	for(int i=0;i<2;i++){
	i2c_burst_read(dev_bmp280,BMP280_ADDR,calib_unsigned_registers[i],&calib_data[0],2);
	calib_data_unsigned_registers[i] = ((uint16_t)((uint16_t)(calib_data[1]))<<8 | ((uint16_t)(calib_data[0])));
	printk("%d\t %d\t %d\t %d\t %x\n",i,calib_data[0],calib_data[1],calib_data_unsigned_registers[i],calib_unsigned_registers[i]);
	calib_data[0]=0;calib_data[1]=0;
	}

	// For loop to get the signed data dig_T2-T3 and dig_P[2-9].
	for(int i=0;i<10;i++){
	i2c_burst_read(dev_bmp280,BMP280_ADDR,calib_signed_registers[i],&calib_data[0],2);
	calib_data_signed_registers[i] = ((int16_t)((int16_t)(calib_data[1]))<<8 | ((int16_t)(calib_data[0])));
	printk("%d\t %d\t %d\t %d\t %x\n",i,calib_data[0],calib_data[1],calib_data_signed_registers[i],calib_signed_registers[i]);
	calib_data[0]=0;calib_data[1]=0;
	}
}

/**
* FUNCTION: To convert the raw data into human readable format. Adapted from the adafruit library.
*/

int32_t temperature_read_raw_data(void){
	int32_t rawdata;
	i2c_burst_read(dev_bmp280,BMP280_ADDR,0xFA,&temp_data[0],3);		// Burst read the temperature data.
	rawdata = (int32_t) ((((uint32_t) (temp_data[0])) << 12) | (((uint32_t) (temp_data[1])) << 4) | ((uint32_t) temp_data[2] >> 4));
	// return uint32_t(temp_data[0])<<16 | uint32_t(temp_data[1])<<8 | uint32_t(temp_data[2]);
	return rawdata;		
}
int32_t read_temperature(void){
	int32_t var1, var2;

	int32_t temp = temperature_read_raw_data();

	var1 = ((((temp >> 3) - ((int32_t)calib_data_unsigned_registers[0] << 1))) * ((int32_t) calib_data_signed_registers[0])) >>11;

	var2 = (((((temp >> 4) - ((int32_t) calib_data_unsigned_registers[0])) * ((temp >> 4) - ((int32_t)calib_data_unsigned_registers[0]))) >> 12) * ((int32_t) calib_data_signed_registers[1])) >> 14;
	t_fine = var1+var2;

	int32_t T = (t_fine  * 5 + 128) >>8;
	return T;
}


/**
*	FUNCTION: To get the raw data of pressure.
*/
int32_t pressure_read_raw_data(void){
	int32_t rawdata;
	i2c_burst_read(dev_bmp280, BMP280_ADDR, 0xF7, &press_data[0],3);
	rawdata = (int32_t) ((((uint32_t) (press_data[0])) << 12) | (((uint32_t) (press_data[1])) << 4) | ((uint32_t) press_data[2] >> 4));
	return rawdata;
}

/**
* FUNCTION: To convert the raw data into human readable format. Adapted from the adafruit library.
*/

int32_t read_pressure(void){
	int64_t var1, var2, p;
	read_temperature();																			// Required to get the t_fine variable.
	int32_t press = pressure_read_raw_data();

	var1 = ((int64_t)t_fine) - 128000;

	var2 = var1 * var1 *(int64_t) calib_data_signed_registers[6];
	var2 = var2 + ((var1 * (int64_t) calib_data_signed_registers[5]) << 17);
	var2 = var2 + (((int64_t) calib_data_signed_registers[4]) << 35);
	var1 = ((var1 * var1 * (int64_t)calib_data_signed_registers[3]) >> 8) + 
					((var1 * (int64_t)calib_data_signed_registers[2]) << 12);

	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data_unsigned_registers[1]) >> 33;

	if(var1 == 0)
		return 0;

	p = 1048576 - press;
	p = (((p<<31) - var2) * 3125) / var1;
	var1 = (((int64_t)calib_data_signed_registers[9]) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)calib_data_signed_registers[8]) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t) calib_data_signed_registers[7]) << 4);
	printk("Value of p: %lld\n",p);
	return p;
}

/**
* FUNCTION: Main function!
*/

void main(void){

	dev_bmp280 = device_get_binding(I2C_DEV);
	const struct device  *dev_usb;																										// Device for USB Console.
	uint32_t dtr=0;
	uint8_t control = 182;																														// Binary 0b10110110 equivalent to 182 in decimal. Pressure_Sampling: 0x16 and Tempreature_Sampling: 0x16.

	dev_usb = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);
	if(usb_enable(NULL))
		return;
	
	while(!dtr)
		uart_line_ctrl_get(dev_usb,UART_LINE_CTRL_DTR, &dtr);	
	  
  if (dev_bmp280 == NULL) {
    printk("I2C: Device driver not found\n");
  }

  else{
    printk("I2C: Device driver found!!!\n"); 
	read_calibration_registers();

    i2c_configure(dev_bmp280, I2C_SPEED_SET(I2C_SPEED_STANDARD));
   
	i2c_reg_read_byte(dev_bmp280,0x77,0xD0,&data_bmp280);															
	if(data_bmp280==88)
		printk("Bosch BMP280 Found!\n");

  }
	 i2c_reg_write_byte(dev_bmp280,BMP280_ADDR,0xF5,0);
	
	while(1){
	i2c_reg_write_byte(dev_bmp280,BMP280_ADDR,0xF4,control);  
	k_sleep(K_MSEC(500));	
	int32_t final_temperature = read_temperature();							
	int32_t final_pressure = read_pressure();
	int32_t dec = final_temperature%100;	
	printk("Tempreature data: %d.%2d\t Pressure: %d\n",final_temperature/100,dec,final_pressure/256); 
	}
  return;
}
