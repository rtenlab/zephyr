/*
 * Copyright (c) 2018 Tavish Naruka <tavishnaruka@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * More data_proximity about the apds9960 sensor registers can be find on the datasheet: https://cdn-learn.adafruit.com/assets/assets/000/045/848/original/Avago-APDS-9960-datasheet.pdf?1504034182
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
#define mask(x) (1<<x)									// Macro for masking bits.
#define DEBUG 0

uint8_t APDS_ADDR = 0x39;								// Register address for apds9960 on i2c Channel
uint8_t APDS_PDATA = 0x9c;								// Register value to get the Proximity data.

uint8_t APDS_CDATAH = 0x95;								//Register address for Clear data channel high byte.
uint8_t APDS_CDATAL = 0x94;								//Register address for Clear data channel low byte.

uint8_t APDS_RDATAL = 0x96;								//Register address for Red data channel high byte.
uint8_t APDS_RDATAH = 0x97;								//Register address for Red data channel low byte.

uint8_t APDS_GDATAL = 0x98;								//Register address for Green data channel high byte.
uint8_t APDS_GDATAH = 0x99;								//Register address for Green data channel low byte.

uint8_t APDS_BDATAL = 0x9A;								//Register address for Blue data channel high byte.
uint8_t APDS_BDATAH = 0x9B;								//Register address for Blue data channel high byte.

uint8_t data_proximity=1;								// Data variable to store the data_proximity from the register.
uint8_t data_L=1;										// Data variable to store the low byte of ALS data.
uint8_t data_H=1;										// Data variable to store the high byte of ALS data.
uint16_t data_c=0,data_r=0,data_g=0,data_b=0;			// 16 bit variable to store tje low and high byte of ALS data
const struct device *dev_apds9960;						// Device struct to get device binding for I2C

/**
* FUNCTION: To read the proximity data_proximity from the sensor. 
*/
void read_proximity_data(){
	i2c_reg_read_byte(dev_apds9960,APDS_ADDR,APDS_PDATA,&data_proximity);
	return;
}

/**
* FUNCTION: To read the clear, red, blue, and green channel for the Ambient Light Sensing.
*/
void read_als_data(){
	// Read the clear channel!
	i2c_reg_read_byte(dev_apds9960,APDS_ADDR,APDS_CDATAL,&data_L);
	i2c_reg_read_byte(dev_apds9960,APDS_ADDR,APDS_CDATAH,&data_H);
	//printk("CDATAL: %d\t CDATAH: %d\n",data_L,data_H);
	data_c = data_H;
	data_c = (data_c<<8);
	data_c |=  data_L;

	// Read the red channel!
	i2c_reg_read_byte(dev_apds9960,APDS_ADDR,APDS_RDATAL,&data_L);
	i2c_reg_read_byte(dev_apds9960,APDS_ADDR,APDS_RDATAH,&data_H);
	//printk("RDATAL: %d\t RDATAH: %d\n",data_L,data_H);
	data_r = data_H;
	data_r = (data_r<<8);
	data_r |=  data_L;

	// Read the Green Channel!
	i2c_reg_read_byte(dev_apds9960,APDS_ADDR,APDS_GDATAL,&data_L);
	i2c_reg_read_byte(dev_apds9960,APDS_ADDR,APDS_GDATAH,&data_H);
	//printk("GDATAL: %d\t GDATAH: %d\n",data_L,data_H);
	data_g = data_H;
	data_g = (data_g<<8);
	data_g |=  data_L;

	//Read the blue channel!
	i2c_reg_read_byte(dev_apds9960,APDS_ADDR,APDS_BDATAL,&data_L);
	i2c_reg_read_byte(dev_apds9960,APDS_ADDR,APDS_BDATAH,&data_H);
	//printk("CDATAL: %d\t CDATAH: %d\n",data_L,data_H);
	data_b = data_H;
	data_b = (data_b<<8);
	data_b |=  data_L;

	return;
}


void main(void){

	dev_apds9960 = device_get_binding(I2C_DEV);
	const struct device  *dev_usb;						// Device for USB Console.
	uint32_t dtr=0;

	dev_usb = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);
	if(usb_enable(NULL))
		return;
	
	while(!dtr)
		uart_line_ctrl_get(dev_usb,UART_LINE_CTRL_DTR, &dtr);	
	  
  if (dev_apds9960 == NULL) {
    printk("I2C: Device driver not found\n");
  }

  else{
    printk("I2C: Device driver found!!!\n"); 

    i2c_configure(dev_apds9960, I2C_SPEED_SET(I2C_SPEED_STANDARD));
   
	i2c_reg_write_byte(dev_apds9960,APDS_ADDR,0x80, mask(0) | mask(2) | mask(1));			// This command the configure the APDS9960 sensor and enables the proximity and ALS property on the device.
	k_sleep(K_MSEC(100));
	
	while(1){
		read_proximity_data();
		read_als_data();
    	printk("I2C: \nProximity:%u\nC:%d\tR:%d\tG:%d\tB:%d\n", 255-data_proximity,data_c,data_r,data_g,data_b);
		k_sleep(K_MSEC(500));

	}
    
  }
}