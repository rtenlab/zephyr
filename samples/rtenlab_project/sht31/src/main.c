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
#define mask(x) (1<<x)																				// Macro for masking bits.


#define STATUS_REG 0xF32D
#define ENABLE_HEATER 0x306D
#define SOFT_RESET 0x30A2
#define HIGH_MEAS_CLOCK_ENABLE 0x2C06
#define MED_MEAS_CLOCK_ENABLE 0x2C0D
#define LOW_MEAS_CLOCK_ENABLE 0x2C10
#define HIGH_MEAS_CLOCK_DIS 02400
#define MED_MEAS_CLOCK_DIS 0x240B
#define LOW_MEAS_CLOCK_DIS 0x2416


const struct device *dev_sht31;															// Device struct to get device binding for I2C
uint16_t SHT31_ADDR = 0x44;																		// Address of the Sensor on I2C bus as described by Adafruit website

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

void configure_device(void){
	dev_sht31 = device_get_binding(I2C_DEV);
	i2c_configure(dev_sht31, I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if (dev_sht31 == NULL) {
		printk("I2C: Device driver not found\n");
	}
	else{
		printk("I2C: Device driver found!!!\n");
	}
	return;
}



uint8_t status_register[6] = {0xF3,0x2D,0x30,0x6D,0x30,0xA2};
/**
* status_register[0] = Read Status Register
* status_register[2] = Enable Register
* status_register[3] = Soft Reset
*
*/

void write_command(uint16_t command){
	uint8_t cmd[2];
	cmd[0] = command>>8;
	cmd[1] = command & 0xFF;
	printk("cmd[0]: %x \t cmd[1]: %x\n",cmd[0],cmd[1]);
	int ret = i2c_write(dev_sht31, &cmd[0], 2, SHT31_ADDR);

	if(ret)
		printk("WRITE UNSUCCESSFULL!!\n");

	return;
}

uint16_t read_status(void){
	int ret;
	uint8_t data[2];
	uint16_t status;
	write_command(STATUS_REG);
	ret = i2c_read(dev_sht31, &data[0], 2, SHT31_ADDR);
		if(ret)
			printf("Unsuccessful Read attempt\n");
	status = data[0];
	status <<= 8;
	status |= data[1];
	return status;
}

void main(void){
	
	enable_uart_console();
	configure_device();

	k_sleep(K_MSEC(50));
	//Enabling Heater
	write_command(ENABLE_HEATER);
	printk("Before Status Register: %d\n",read_status());
	write_command(SOFT_RESET);
	k_sleep(K_MSEC(50));

	//ACtually reading status register
	printk("After Status Register: %d\n",read_status());
	return;
}
