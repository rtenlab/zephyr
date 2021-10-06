/*
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
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


uint8_t data_lis=1;
const struct device *dev_sht31;															// Device struct to get device binding for I2C
uint8_t SHT31_ADDR = 0x77;																		// Address of the Sensor on I2C bus as described by Adafruit website



// CHANGES NEEDS TO BE DONE ON THIS PART. THIS IS JUST A COPY FROM THE LIS3MDL.

void main(void){

	dev_sht31 = device_get_binding(I2C_DEV);
	const struct device  *dev_usb;																										// Device for USB Console.
	uint32_t dtr=0;
	

	dev_usb = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);
	if(usb_enable(NULL))
		return;
	
	while(!dtr)
		uart_line_ctrl_get(dev_usb,UART_LINE_CTRL_DTR, &dtr);	
	  
  if (dev_sht31 == NULL) {
    printk("I2C: Device driver not found\n");
  }

  else{
    printk("I2C: Device driver found!!!\n"); 


    i2c_configure(dev_sht31, I2C_SPEED_SET(I2C_SPEED_STANDARD));
   
	i2c_reg_read_byte(dev_sht31,0x1C,0x0F,&data_lis);															// This command the configure the APDS9960 sensor and enables the proximity and ALS property on the device.
	if(data_lis==61)
		printk("Sensor found Found!\n");

  }	
  return;
}
