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
#define mask(x) (1<<x)									// Macro for masking bits.
#define DEBUG 0

uint8_t data_bmp280=1;
const struct device *dev_bmp280;						// Device struct to get device binding for I2C
uint8_t BMP280_ADDR = 0x77;
uint8_t temp_data[2];
uint8_t press_data[2];
uint32_t temperature, pressure;
uint8_t calib_data[2];
uint16_t dig_uns[2];	//Contains dig_T1 and dig_P1 register.
int16_t dig_signed[10];	// Containes dig_T2,T3 and dig_[P2-P9] registers.
uint8_t dig_regs[10]= {0x8A,0x8C,0x90,0x92,0x94,0x96,0x98,0x9A,0x9C,0x9E};
uint8_t dig_16_regs[2] = {0x88,0x8E};
uint16_t dig_try=0;

void read_calibration_registers(void){
	printk("I\t calib[0]\t calib[1]\t final\t Register\n");
	for(int i=0;i<2;i++){
	i2c_burst_read(dev_bmp280,BMP280_ADDR,dig_16_regs[i],&calib_data[0],2);
	dig_uns[i] = ((uint16_t)((uint16_t)(calib_data[1]))<<8 | ((uint16_t)(calib_data[0])));
	printk("%d\t %d\t %d\t %d\t %x\n",i,calib_data[0],calib_data[1],dig_uns[i],dig_16_regs[i]);
	calib_data[0]=0;calib_data[1]=0;
	}
	for(int i=0;i<10;i++){
	i2c_burst_read(dev_bmp280,BMP280_ADDR,dig_regs[i],&calib_data[0],2);
	dig_signed[i] = ((int16_t)((int16_t)(calib_data[1]))<<8 | ((int16_t)(calib_data[0])));
	printk("%d\t %d\t %d\t %d\t %x\n",i,calib_data[0],calib_data[1],dig_signed[i],dig_regs[i]);
	calib_data[0]=0;calib_data[1]=0;
	}
	i2c_burst_read(dev_bmp280,BMP280_ADDR,0x96,&calib_data[0],2);
	printk("%d \t %d\n",calib_data[0],calib_data[1]);
	dig_try = ((uint16_t)((uint16_t)(calib_data[1]))<<8 | ((uint16_t)(calib_data[0])));
	printk("dig_try: %d\n",dig_try);
}
void main(void){

	dev_bmp280 = device_get_binding(I2C_DEV);
	const struct device  *dev_usb;						// Device for USB Console.
	uint32_t dtr=0;

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
   
	i2c_reg_read_byte(dev_bmp280,0x77,0xD0,&data_bmp280);			// This command the configure the APDS9960 sensor and enables the proximity and ALS property on the device.
	if(data_bmp280==88)
		printk("Bosch BMP280 Found!\n");
  }
	 i2c_reg_write_byte(dev_bmp280,BMP280_ADDR,0xF5,0);
	
	while(1){
	i2c_reg_write_byte(dev_bmp280,BMP280_ADDR,0xF4,37);  
	k_sleep(K_MSEC(500));

	/**	EARLIER IMPLEMENTATION OF READING THE TEMPREATURE IN BURST MODE.
	// for(int i=0;i<2;i++){
	// 	i2c_reg_read_byte(dev_bmp280,BMP280_ADDR,temp_regs[i],&temp_data[i]);
	// 	i2c_reg_read_byte(dev_bmp280,BMP280_ADDR,press_regs[i],&press_data[i]);
	// }
	*/
	i2c_burst_read(dev_bmp280,BMP280_ADDR,0xFA,&temp_data[0],2);		// Burst read the temperature data.
	i2c_burst_read(dev_bmp280,BMP280_ADDR,0xF7,&press_data[0],2);		// Burst read the pressure data.
	//value = (uint32_t)((((uint32_t) (temp_data[0]))<<12) | (((uint32_t) (temp_data[1])) <<4) | ((uint32_t) temp_data[2]>>4));
	temperature = ((uint32_t)((uint32_t)(temp_data[0]))<<8 |(uint32_t)(temp_data[1]));
	pressure = ((uint32_t)((uint32_t)(press_data[0]))<<8 |(uint32_t)(press_data[1]));
	printk("Tempreature data: %d\t Presssure Data: %d\n\n",temperature,pressure); 
	}

  return;
}