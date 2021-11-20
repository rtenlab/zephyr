/*
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * Adafruit I2C support library: https://github.com/adafruit/Adafruit_BusIO
 * Zephyr I2C documentation: https://docs.zephyrproject.org/latest/reference/peripherals/i2c.html
 */


/** 
 * Cable configuration:
 * Green-cable: SDA
 * Red cable: VDD
 * Yellow cable: SCL
 * Black cable: GND
 * I2C ADDRESS FOR THE SENSOR IS 0X62
 */

/**
 * @file: This app enables the use of SCD41 CO2 Sensor  sensor on zephyr OS.
 */
#include<stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>


#define I2C_DEV "I2C_0"

const struct device *dev_i2c;															// Device struct to get device binding for I2C
uint16_t SCD41_ADDR = 0x62;															// Address of the Sensor on I2C bus as described by Adafruit website

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF

uint8_t sensirion_common_generate_crc(const uint8_t* data, uint16_t count) {
uint16_t current_byte;
uint8_t crc = CRC8_INIT;
uint8_t crc_bit;
/* calculates 8-Bit checksum with given polynomial */
for (current_byte = 0; current_byte < count; ++current_byte) {
crc ^= (data[current_byte]);
for (crc_bit = 8; crc_bit > 0; --crc_bit) {
if (crc & 0x80)
crc = (crc << 1) ^ CRC8_POLYNOMIAL;
else
crc = (crc << 1);
}
}
return crc;
}

uint16_t sensirion_i2c_add_command_to_buffer(uint8_t* buffer, uint16_t offset,
                                             uint16_t command) {
    buffer[offset++] = (uint8_t)((command & 0xFF00) >> 8);
    buffer[offset++] = (uint8_t)((command & 0x00FF) >> 0);
    return offset;
}

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
	dev_i2c = device_get_binding(I2C_DEV);
	i2c_configure(dev_i2c, I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if (dev_i2c == NULL) {
		printk("I2C: Device driver not found\n");
	}
	else{
		printk("I2C: Device driver found!!!\n");
	}
	return;
}
void main(void){
	// enabling uart_console for zephyr.
	enable_uart_console();

	// configuring the i2c device for communication with the sensor.
	configure_device();
	uint32_t num_bytes = 3;
	uint8_t write[3] = {0x36,0x82};	
	uint8_t data[9];
	uint8_t crc = sensirion_common_generate_crc(&write[0], 2);
	printf("%d\n",crc);
	write[2] = crc;
	int ret = i2c_write(dev_i2c, &write[0], num_bytes, SCD41_ADDR);
	printf("i2c_write: [%d]\n", ret);
	k_sleep(K_MSEC(3));
	ret = i2c_read(dev_i2c, &data[0], 9, SCD41_ADDR);
	printf("i2c_read: [%d]\n", ret);
	for(int i=0; i<9; i++)
		printf("%d\t %2x\n",data[i], data[i]);
	return;
}
