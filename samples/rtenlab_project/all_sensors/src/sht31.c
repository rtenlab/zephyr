#include "sht31.h"

const struct device *dev_i2c;														// Device struct to get device binding for I2C
uint16_t SHT31_ADDR = 0x44;																		// Address of the Sensor on I2C bus as described by Adafruit website



/**
* @FUNCTION: API to write commands to the sensor using i2c_write() API provided in zephyr.
*/
void write_command(uint16_t command){
	//Convert 16 bit data address into chunk of two 8-bit data address and write to I2C in burst mode.
	uint8_t cmd[2];
	cmd[0] = command>>8;
	cmd[1] = command & 0xFF;
	int ret = i2c_write(dev_i2c, &cmd[0], 2, SHT31_ADDR);
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
	ret = i2c_read(dev_i2c, &data[0], 2, SHT31_ADDR);
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
void read_temp_hum(sht31_t *var){
	uint8_t read_buffer[6];
	write_command(HIGH_MEAS_CLOCK_ENABLE);
	k_sleep(K_MSEC(50));
	int ret = i2c_read(dev_i2c, &read_buffer[0], 6, SHT31_ADDR);
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

void print_data_sht(sht31_t* data){
    printk("Temp: %d.%2d\t Humidity: %d.%2d\n",data->temp.integer, data->temp.frac, data->humidity.integer, data->humidity.frac);
    return;
}