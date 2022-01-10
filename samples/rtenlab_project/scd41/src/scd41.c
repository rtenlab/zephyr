#include "scd41.h"
 
const struct device *dev_i2c; 
      // Device struct to get device binding for I2C
uint16_t SCD41_ADDR = 0x62;		// Address of the Sensor on I2C bus as described by Adafruit website
uint8_t command_to_sensor[NUM_BYTES_TO_SENSOR];



/**
 * FUNCTION: function to get the crc checksum for the output command.
 * Adapted from Sensirion lbrary: https://github.com/Sensirion/embedded-i2c-scd4x
 */
/* calculates 8-Bit checksum with given polynomial */
uint8_t generate_crc(const uint8_t* data, uint16_t count) {
    uint16_t current_byte;
    uint8_t crc = CRC8_INIT;
    uint8_t crc_bit;
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


/**
 * @FUNCTION: Function to format the output command that needs to be sent to sensor.
 */
void get_formatted_command(uint16_t command){
	//printf("Preparing 0x%x command to the global buffer!!!\n", command);
	command_to_sensor[0] = (uint8_t)((command & 0xFF00) >> 8);
	command_to_sensor[1] = (uint8_t)((command & 0x00FF) >> 0);
	uint8_t crc = generate_crc(&command_to_sensor[0],2);
	command_to_sensor[2] = crc;	
	return;
}


/**
 * @FUNCTION: function to get the serial number of the sensor.
 */
void get_serial_number(void){
	uint8_t data[9];			// temporary buffer to store the data from the sensor.
	uint16_t serial[3];			// buffer to store just the serial number, excluding crc check.
	get_formatted_command((uint16_t)GET_SERIAL_NUMBER);		// prepares output command in a buffer with the CRC check.
	int ret = i2c_write(dev_i2c, &command_to_sensor[0], NUM_BYTES_TO_SENSOR, SCD41_ADDR);	// write the command prepared in the previous function call to the sensor device. 

	if(ret!=0)
		printf("Error while writing to the device: %d\n", ret);
	delay(3);					//wait for 3 Msec for the sensor to process the command.
	ret = i2c_read(dev_i2c, &data[0],9, SCD41_ADDR);		//read the data sent from the sensor including crc checksum bits.

	if(ret !=0)
		printf("Error while reading from the device: %d\n", ret);
	
	// Get the 3 words in different 16 bit entries in serial.
	serial[0] = data[0] <<8 | data[1];
	serial[1] = data[3] << 8 | data[4];
	serial[2] = data[6] << 8 | data[7];

	//print the output data.
	printf("Correct Serial ID: 0x%4X%4X%4X\n",serial[0],serial[1], serial[2]);

	return;
}

/**
 * FUNCTION: function to read the measured data from the sensor.
 */ 
void read_sensor_data(uint8_t *data, int num_bins_to_read){
	// Command to notify sensor requirement of data.
	get_formatted_command((uint16_t)READ_MEASUREMENT);
	i2c_write(dev_i2c, &command_to_sensor[0], NUM_BYTES_TO_SENSOR, SCD41_ADDR);
	delay(3);
	// Read 9 bytes of data. In order: CO2 -> Tempreature -> humidity.
	i2c_read(dev_i2c, data, num_bins_to_read, SCD41_ADDR);
	return;
}

/**
 * FUNCTION: function to measure data once from the sensor.
 */
void measure_single_shot(scd41_t* data){
	// buffers to store the incoming data from the sensor.
	uint8_t data_buffer[9];
	uint16_t format_data=0;
	// Command to notify sensor to measure data just once.
	get_formatted_command((uint16_t)MEASURE_SINGLE_SHOT);
	i2c_write(dev_i2c, &command_to_sensor[0], NUM_BYTES_TO_SENSOR, SCD41_ADDR);
	//delay of 5 sec for the sensor to get data.
	delay(5000);
	// function call to read 9 bytes of data from the sensor.
	read_sensor_data(&data_buffer[0], 9);
	//store the CO2 data coming from the sensor directly to struct.
	data->Co2 = data_buffer[0] <<8 | data_buffer[1];
	// store the tempreature data coming from the sensor with some modifications as per datasheet.
	format_data = data_buffer[3] << 8 | data_buffer[4]; 
	data->temp = (float)(format_data*175.0/65536.0-45.0);
	// store the humidity data coming from the sensor with some modifications as per datasheet.
	format_data = data_buffer[6] << 8 | data_buffer[7];
	data->hum = (float)(format_data*100.0/65536.0);	
	return;
}

/**
 * FUNCTION: function to print the data obtained from the sensor.
 */
void print_data_scd(scd41_t* data){
    	printf("CO2:%d\t Hum:%f\t Temp: %f\n", data->Co2, data->hum, data->temp);
	return;
}