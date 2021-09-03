/*
 * Copyright (c) 2018 Tavish Naruka <tavishnaruka@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * More data about the apds9960 sensor registers can be find on the datasheet: https://cdn-learn.adafruit.com/assets/assets/000/045/848/original/Avago-APDS-9960-datasheet.pdf?1504034182
 */

#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>


/**
 * @file This app enables the proximity bit on the apds9960 sensor and reads the value from a register specified for reading the proximity data. It also establishes a usb console for reading the printk statements.
 */


#define I2C_DEV "I2C_0"
#define mask(x) (1<<x)

uint8_t APDS_ADDR = 0x39;
uint8_t APDS_PDATA = 0x9c;
uint8_t data=1;
const struct device *dev_apds9960;
void read_proximity_data(){
	i2c_reg_read_byte(dev_apds9960,APDS_ADDR,APDS_PDATA,&data);
}
void main(void){

  dev_apds9960 = device_get_binding(I2C_DEV);
  const struct device  *dev_usb;
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
   
	i2c_reg_write_byte(dev_apds9960,APDS_ADDR,0x80, mask(0) | mask(2));

	k_sleep(K_MSEC(100));
	// i2c_reg_read_byte(dev_apds9960,APDS_ADDR,0x80, &data);
	while(1){
		read_proximity_data();
    	printk("I2C: Proximity:%u\n", 255-data);
		k_sleep(K_MSEC(100));

	}
    
  }
}