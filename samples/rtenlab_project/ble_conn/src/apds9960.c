#include "uart_i2c.h"
#include "apds9960.h"

// const struct device *dev_i2c;

apds9960_t apds9960_data;

/**
* FUNCTION: To read the clear, red, blue, and green channel for the Ambient Light Sensing.
*/
void read_als_data(apds9960_t* data){
	uint8_t data_buffer[8];
	// Read the clear channel!
	i2c_burst_read(dev_i2c, APDS_ADDR, APDS_CDATAL, &data_buffer[0], 8);
	data->clear = data_buffer[1] << 8 | data_buffer[0];
	// Read the red channel!
	data->red = data_buffer[3]<<8| data_buffer[2];
	// Read the Green Channel!
	data->green = data_buffer[5] << 8 | data_buffer[4];
    //Read the blue channel!
	data->blue = data_buffer[7] << 8 | data_buffer[6];
	return;
}

/**
* FUNCTION: To read the proximity data_proximity from the sensor. 
*/
void read_proximity_data(apds9960_t* data){
    i2c_reg_read_byte(dev_i2c,APDS_ADDR,APDS_PDATA,&(data->proximity));
    return;
}

/**
* FUNCTION: To enable the apds sensor to collect   from the sensor. 
*/
void enable_apds_sensor(void){
	i2c_reg_write_byte(dev_i2c,APDS_ADDR,APDS_ENABLE, mask(0) | mask(2) | mask(1));			// This command the configure the APDS9960 sensor and enables the proximity and ALS property on the device.
	delay(100);
    return;

}

void print_data_apds(apds9960_t* data){
	printk("Proximity: %d\nClear: %d\t Red: %d\t Green: %d\t Blue: %d\n", 255-data->proximity, data->clear, data->red, data->blue, data->green);
	return;
}