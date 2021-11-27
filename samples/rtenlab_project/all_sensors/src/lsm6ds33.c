#include "lsm6ds33.h"


uint16_t LSM6DS_ADDR = 0x6A;															// Address of the Sensor on I2C bus as described by Adafruit website

int16_t data_buffer[7];	// Buffer to hold the 16-bit data for each paramter in gyroscope and accel.
float temperature_sensitivity = 256.0;
float accel_scale_4_G = 0.122;
float gyro_scale_1000_dps = 35.0;



void check_device(void){
	uint8_t device_id=0;
	i2c_reg_read_byte(dev_i2c, LSM6DS_ADDR, 0X0F, &device_id);
	if(device_id == DEV_ID)
		printk("LSM6DS33 device found!\n");
}

/**
 * @FUNCTION: This function sets the CTRL1_XL register on the sensor to get the accel data rate to 104Hz and 4G data range.
 */
void set_acc_data_rate_range(void){
	int ret = i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, CTRL1_XL, ACC_104_HZ_4G); 	
	if(ret != 0)
		printk("Error while setting the ACC_range!\n");
	return;
}

/**
 * @FUNCTION: This function sets the CTRL2_G register on the sensor to get the accel data rate to 104Hz and 1000dps data range.
 */
void set_gyro_data_rate_range(void){
	int ret = i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, CTRL2_G, GYRO_104_HZ_1000dps); 	
	if(ret!=0)
		printk("Error while setting the gyro_range!\n");
	return;
}

/**
 * @FUNCTION: This function pools the status of the sensor by reading the status register.
 */
void status_reg(void){
	uint8_t status=0;
	int ret = i2c_reg_read_byte(dev_i2c, LSM6DS_ADDR, STATUS_REG_LSM, &status);
	if(ret!=0)
		printk("Error reading the status of the register\n");
	printk("Status: %d\n", status);
	return;
}

/**
 * @FUNCTION: This function gets the data from the sensor in bursts.
 */
void read_burst_data(lsm6ds33_t *data){
	uint8_t buffer[14];
	i2c_burst_read(dev_i2c, LSM6DS_ADDR, OUT_TEMP_L, &buffer[0], 14);
	for(int i=0; i<13; i+=2){
		data_buffer[i/2] = buffer[i+1] << 8 | buffer[i];
//		printf("data_buffer[%d] = buffer[%d] << 8 | buffer[%d]\n", i/2, i+1, i);
	}
	data->gyroX = data_buffer[1] * gyro_scale_1000_dps *  SENSORS_DPS_TO_RADS/1000.0;
	data->gyroY = data_buffer[2] * gyro_scale_1000_dps *  SENSORS_DPS_TO_RADS/1000.0;
	data->gyroZ = data_buffer[3] * gyro_scale_1000_dps *  SENSORS_DPS_TO_RADS/1000.0;

	data->accelX = data_buffer[4] * accel_scale_4_G * SENSORS_GRAVITY_STANDARD / 1000;
	data->accelY = data_buffer[5] * accel_scale_4_G  *SENSORS_GRAVITY_STANDARD / 1000;
	data->accelZ = data_buffer[6] * accel_scale_4_G  *SENSORS_GRAVITY_STANDARD / 1000;
	return;
}

void print_data_lsm(lsm6ds33_t *data){
	printf("Gyro: %f\t %f\t %f  dps\n",data->gyroX, data->gyroY, data->gyroZ);
	printf("Accel: %f\t %f\t %f\n",data->accelX, data->accelY, data->accelZ);
	return;
}

void lsm6ds33_init(void){
	set_gyro_data_rate_range();
	set_acc_data_rate_range();

}