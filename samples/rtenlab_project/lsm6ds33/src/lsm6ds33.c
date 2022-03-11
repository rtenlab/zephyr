#include "lsm6ds33.h"

const struct device *dev_i2c;															// Device struct to get device binding for I2C
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
 * @FUNCTION: This function sets the CTRL1_XL register on the sensor to get the accel data rate to 26Hz and 4G data range.
 */
void set_acc_data_rate_range(void){
	int ret = i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, CTRL1_XL, ACC_26_HZ_4G); 	
	if(ret != 0)
		printk("Error while setting the ACC_range!\n");
	return;
}

/**
 * @FUNCTION: This function sets the CTRL2_G register on the sensor to get the accel data rate to 26Hz and 1000dps data range.
 */
void set_gyro_data_rate_range(void){
	int ret = i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, CTRL2_G, GYRO_26_HZ_1000dps); 	
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
 * @brief  Get the CTRL1_Xl register and we can infer the frequency and sensitivity of the data collected.
 * @note   
 * @retval None
 */
void get_accel_rate(void){
	uint8_t rate=0;
	int ret = i2c_reg_read_byte(dev_i2c, LSM6DS_ADDR, CTRL1_XL, &rate);
	if(ret!=0)
		printk("Error reading the status of the register\n");
	printk("Rate: 0x%x\n", rate);
	return;
}

/**
 * @brief  Set the power mode for Accelerometer.
 * @note   
 * @param  *data: 
 * @retval None
 */
void accel_set_power_mode(uint8_t power){
	int ret = i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, CTRL6_ACCEL, power); 	
	if(ret!=0)
		printk("Error while setting the gyro_range!\n");
	if(power==0){
		printk("High Performance mode set\n");
	}
	else
		printk("Low Power Mode set\n");
	return;
}


/**
 * @brief  Set the power mode for gyro.
 * @note   
 * @param  *data: 
 * @retval None
 */
void gyro_set_power_mode(uint8_t power){
	int ret = i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, CTRL6_GYRO, power); 	
	if(ret!=0)
		printk("Error while setting the gyro_range!\n");
	if(power==0){
		printk("High Performance mode set\n");
	}
	else
		printk("Low Power Mode set\n");
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

void print_data(lsm6ds33_t *data){
	printf("Gyro: %f\t %f\t %f  dps\n",data->gyroX, data->gyroY, data->gyroZ);
	printf("Accel: %f\t %f\t %f\n",data->accelX, data->accelY, data->accelZ);
	return;
}

void lsm6ds33_init(void){
	set_gyro_data_rate_range();
	set_acc_data_rate_range();
	fifo_t setting;
	// 001 -> No decimation.
	setting.axel_decimation_rate = 1; 
	setting.gyro_decimation_rate = 1;
	// 110 -> Continuous mode
	setting.mode = 6;

	//26 HZ. Same as the current gyro and accel data.
	setting.sample_rate = 2;

	// Not really sure about this.
	setting.threshold = 1;

	lsm6ds33_fifo_begin(&setting);
	lsm6ds33_fifo_clear();
}


void lsm6ds33_fifo_begin(fifo_t* setting){
	uint8_t threshold_lsb = (setting->threshold)&0x00FF;
	uint8_t threshold_msb = (setting->threshold)&0x0F00>>8;
	uint8_t temp_decimation_rate = (setting->gyro_decimation_rate&0x07)<<3 | (setting->axel_decimation_rate&0x07);
	printk("final_decimation_rate=%d\n", temp_decimation_rate);
	

	uint8_t temp_ctrl5=(setting->sample_rate&0x0F)<<3 | (setting->mode&0x07);
	printk("Final_mode and odr: %d\n", temp_ctrl5);

	i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, FIFO_CTRL1, threshold_lsb);
	i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, FIFO_CTRL2, threshold_msb);
	i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, FIFO_CTRL3, temp_decimation_rate);
	i2c_reg_write_byte(dev_i2c, LSM6DS_ADDR, FIFO_CTRL5, temp_ctrl5);
}

uint16_t lsm6ds33_fifo_status(void){
	uint8_t buffer[2]; 
	uint16_t tempAcc=0;
	i2c_burst_read(dev_i2c, LSM6DS_ADDR, FIFO_STATUS1, &buffer[0], 2);
	
	tempAcc = (buffer[1]<<8) | (buffer[0]);
	printk("Inside status: %d\n", tempAcc&0xF00);
	return tempAcc;

}

uint16_t lsm6ds33_fifo_read(void){
	uint8_t buffer[2];
	// read the first 16 bits of data and send it as return value.
	i2c_burst_read(dev_i2c, LSM6DS_ADDR, FIFO_DATA_OUT_L, &buffer[0], 2);
	uint16_t tempAcc =(uint16_t)((buffer[1]<<8) | (buffer[0]));

	return tempAcc;
}

void lsm6ds33_fifo_clear(void){
	// The last fourth bit of the returned value will be FIFO_EMPTY flag. Check the flag until we reach it. 
	while(lsm6ds33_fifo_status() &0x1000){
		lsm6ds33_fifo_read();
	}
	return;
}