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
#define HIGH_MEAS_CLOCK_DIS 0X2400
#define MED_MEAS_CLOCK_DIS 0x240B
#define LOW_MEAS_CLOCK_DIS 0x2416


const struct device *dev_sht31;															// Device struct to get device binding for I2C
uint16_t SHT31_ADDR = 0x44;																		// Address of the Sensor on I2C bus as described by Adafruit website

// Struct to hold the integer and fractional information of temperature and humidity.
typedef struct temperature{
	int32_t integer;
	uint8_t frac;	
}temperature_t;

typedef struct humidity{
	uint32_t integer;
	uint8_t frac;
}humidity_t;

typedef struct temphum{
	temperature_t temp;
	humidity_t humidity;
}temphum_t;



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

/**
* @FUNCTION: API to write commands to the sensor using i2c_write() API provided in zephyr.
*/
void write_command(uint16_t command){
	//Convert 16 bit data address into chunk of two 8-bit data address and write to I2C in burst mode.
	uint8_t cmd[2];
	cmd[0] = command>>8;
	cmd[1] = command & 0xFF;
	int ret = i2c_write(dev_sht31, &cmd[0], 2, SHT31_ADDR);
	if(ret){
		printk("WRITE UNSUCCESSFULL!!\n");
	}
	return;
}

/**
* @FUNCTION: API to read the status of the SHT31 Address. It will return 16 bit data from sensor.
*/
uint16_t read_status(void){
	int ret;
	uint8_t data[2];
	uint16_t status;
	// Give command to read the status register.
	write_command(STATUS_REG);
	// Read the I2C device to get status register data.
	ret = i2c_read(dev_sht31, &data[0], 2, SHT31_ADDR);
		if(ret)
			printf("Unsuccessful Read attempt\n");

	//Manipulate the data to convert the 2 chunks of 8-bit data into 16-bit data.
	status = data[0];
	status <<= 8;
	status |= data[1];
	return status;
}

/**
* @FUNCTION: API to read the temperature and humidity data from SHT31 Sensor. All data is global so no input 
*/
void read_temp_hum(temphum_t *var){
	uint8_t read_buffer[6];
	write_command(HIGH_MEAS_CLOCK_ENABLE);
	k_sleep(K_MSEC(50));
	int ret = i2c_read(dev_sht31, &read_buffer[0], 6, SHT31_ADDR);
	if(ret){
		printk("ret:%d \n",ret);
		printk("UnSUCESSFUL READ TEMPREATURE AND HUMIDITY\n");
	}
		
	int32_t stemp = (int32_t)(((uint32_t)read_buffer[0] << 8) | read_buffer[1]);
	stemp = ((4375 * stemp) >> 14) - 4500;
	var->temp.integer = stemp/100;
	var->temp.frac = stemp%100;

	uint32_t shum = ((uint32_t)read_buffer[3] << 8) | read_buffer[4];
	shum = (625 * shum) >> 12;
	var->humidity.integer = shum/100;
	var->humidity.frac = shum%100;
	return;
}


void main(void){
	temphum_t variable;
	temphum_t* var1=&variable;
	// var1->temp.integer=0;
	// var1->temp.frac=0;
	// var1->humidity.integer=0;
	// var1->humidity.frac=0;

	enable_uart_console();
	configure_device();
	
	while(1){
		read_temp_hum(var1);
		printk("Temp: %d.%2d\t Humidity: %d.%2d\n",var1->temp.integer, var1->temp.frac, var1->humidity.integer, var1->humidity.frac);
		k_sleep(K_MSEC(10));
	}
	
	//ACtually reading status register
	printk("After Status Register: %d\n",read_status());
	return;
}
