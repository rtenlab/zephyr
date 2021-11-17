/*
 * SPDX-License-Identifier: Apache-2.0
 * Adapted from https://devzone.nordicsemi.com/f/nordic-q-a/70098/i2c-twi-master-config-on-zephyr.
 * Adafruit LSM6DS Library: https://github.com/adafruit/Adafruit_LSM6DS
 * Adafruit I2C support library: https://github.com/adafruit/Adafruit_BusIO
 * Zephyr I2C documentation: https://docs.zephyrproject.org/latest/reference/peripherals/i2c.html
 */




/**
 * @file: This app enables the use of LSM6DS33 sensor on zephyr OS.
 */
#include<stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>


#define I2C_DEV "I2C_0"
#define DEV_ID 105
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define STATUS_REG 0x1E
#define OUT_TEMP_L 0x20
#define OUT_TMP_H 0x21

#define ACC_104_HZ 0x20		// Set gyroscope data rate to 104HZ
#define GYRO_104_HZ 0x20	// Set Accel. data rate to 104HZ
// below value can be found on this website: https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h
#define SENSORS_DPS_TO_RADS (0.017453293F)		

const struct device *dev_i2c;															// Device struct to get device binding for I2C
uint16_t LSM6DS_ADDR = 0x6A;															// Address of the Sensor on I2C bus as described by Adafruit website

int16_t data_buffer[7];	// Buffer to hold the 16-bit data for each paramter in gyroscope and accel.
float temperature_sensitivity = 256.0;

typedef struct lsm6ds33{
	uint16_t temp;
	float gyroX,
		  gyroY,
		  gyroZ, 
		  accelX,
		  accelY,
		  accelZ;
}lsm6ds33_t;
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

void check_device(){
	uint8_t device_id=0;
	i2c_reg_read_byte(dev_i2c, LSM6DS_ADDR, 0X0F, &device_id);
	if(device_id == DEV_ID)
		printk("LSM6DS33 device found!\n");
}

void set_acc_data_rate(void){
	int ret = i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, CTRL1_XL, ACC_104_HZ); 	
	if(ret != 0)
		printk("Error while setting the ACC_range!\n");
	return;
}

void set_gyro_data_rate(void){
	int ret = i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, CTRL2_G, GYRO_104_HZ); 	
	if(ret!=0)
		printk("Error while setting the gyro_range!\n");
	return;
}

void status_reg(void){
	uint8_t status=0;
	int ret = i2c_reg_read_byte(dev_i2c, LSM6DS_ADDR, STATUS_REG, &status);
	if(ret!=0)
		printk("Error reading the status of the register\n");
	printk("Status: %d\n", status);
	return;
}

// Need to work on this!

void read_burst_data(lsm6ds33_t *data){
	uint8_t buffer[14];
	i2c_burst_read(dev_i2c, LSM6DS_ADDR, OUT_TEMP_L, &buffer[0], 14);
	for(int i=0; i<13; i+=2){
		data_buffer[i/2] = buffer[i+1] << 8 | buffer[i];
	}
	// data_buffer[0] = buffer[1]<<8 | buffer[8];
	// uint16_t temp = (data_buffer[0]/temperature_sensitivity)+25.0;
	// data_buffer[1] = buffer[3] << 8 | buffer[2];
	// data_buffer[2]= buffer[5] << 8 | buffer[4];
	// data_buffer[3] = buffer[7] << 8 | buffer[6];
	data->gyroX = data_buffer[1] * 8.75 *  SENSORS_DPS_TO_RADS/1000.0;
	data->gyroY = data_buffer[2] * 8.75 *  SENSORS_DPS_TO_RADS/1000.0;
	data->gyroZ = data_buffer[3] * 8.75 *  SENSORS_DPS_TO_RADS/1000.0;
	return;
}

void print_data(lsm6ds33_t *data){
	printf("Gyro: %f\t %f\t %f\n",data->gyroX, data->gyroY, data->gyroZ);
	return;
}

void lsm6ds33_init(void){
	set_gyro_data_rate();
	set_acc_data_rate();

}
void main(void){
	// enabling uart_console for zephyr.
	enable_uart_console();

	// configuring the i2c device for communication with the sensor.
	configure_device();

	// check if we have the sensor we want.
	check_device();

	// Initialize the sensor by setting necessary parameters for gyroscope and accelerometer
	lsm6ds33_init();

	lsm6ds33_t all_data;
	

	while(1){
		
		k_sleep(K_MSEC(90));
		read_burst_data(&all_data);
		print_data(&all_data);
	}

		
	return;
}
