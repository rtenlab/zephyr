/*
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * Adafruit SHT3X Library: https://github.com/adafruit/Adafruit_SHT31/blob/master/Adafruit_SHT31.cpp
 * Adafruit I2C support library: https://github.com/adafruit/Adafruit_BusIO
 * Zephyr I2C documentation: https://docs.zephyrproject.org/latest/reference/peripherals/i2c.html
 */




/**
 * @file: This app enables the use if SHT-31 sensor on zephyr OS.
 */
#include<stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>


#define I2C_DEV "I2C_0"






const struct device *dev;															// Device struct to get device binding for I2C
uint16_t lis3mdl_ADDR = 0x1C;																		// Address of the Sensor on I2C bus as described by Adafruit website


/**
*@FUNCTION: API to configure the uart console for printk statements.
*/
void enable_uart_console(void){
	const struct device  *dev_usb;																										// Device for USB Console.
	uint32_t dtr=0;
	

	dev_usb = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);
	if(usb_enable(NULL))
		return;
	
	while(!dtr)
		uart_line_ctrl_get(dev_usb,UART_LINE_CTRL_DTR, &dtr);	
	  
	return;
}

/**
*@FUNCTION: API to configure the I2C device.
*/
void configure_device(){
	dev = device_get_binding(I2C_DEV);
	printk("Aatlu chalyu\n");
	i2c_configure(dev, I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if (dev == NULL) {
		printk("I2C: Device driver not found\n");
	}
	else{
		printk("I2C: Device driver found!!!\n");
	}
	return;
}



void main(void){
	uint8_t test_value;
	enable_uart_console();
	configure_device();
	uint8_t addr = 0x0F;
	i2c_reg_read_byte(dev, lis3mdl_ADDR, addr, &test_value);
	printk("I am: %d\n", test_value);
	return;
}
