#include "bmp280.h"

uint8_t calib_data[2];																				// dump array of registers to burst read the 16 bit calib data.
uint16_t calib_data_unsigned_registers[2];										//Contains dig_T1 and dig_P1 register.
int16_t calib_data_signed_registers[10];											// Containes dig_T2,T3 and dig_[P2-P9] registers.
uint8_t calib_signed_registers[10]= {0x8A,0x8C,0x90,0x92,
						0x94,0x96,0x98,0x9A,0x9C,0x9E};										// Register address to get the calibaration data.
uint8_t calib_unsigned_registers[2] = {0x88,0x8E};						// Register address to get the unsigned calibration data.

uint8_t temp_data[3];																					// Array to hold the raw temperature data.
uint8_t press_data[3];																				// Array to hold the raw pressure data.
uint32_t temperature, pressure;																// 32 bit variable to convert the 20 bit data from the array of registers to a single variable

int32_t t_fine;


uint8_t check_sensor(void){
    uint8_t temp;
    i2c_reg_read_byte(dev_i2c,BMP280_ADDR,REG_ID,&temp);															
	if(temp==0x58){
		printk("Bosch BMP280 Found!\n");
        return 1;
    }
    else{
        printk("BMP280 Sensor Not Found!\n");
        return 0;
    }
}

void read_calibration_registers(void){
	printk("I\t calib[0]\t calib[1]\t final_temperature\t Register\n");

	// For loop tp get the unsigned data dig_T1 and dig_P1.
	for(int i=0;i<2;i++){
	i2c_burst_read(dev_i2c,BMP280_ADDR,calib_unsigned_registers[i],&calib_data[0],2);
	calib_data_unsigned_registers[i] = ((uint16_t)((uint16_t)(calib_data[1]))<<8 | ((uint16_t)(calib_data[0])));
	printk("%d\t %d\t %d\t %d\t %x\n",i,calib_data[0],calib_data[1],calib_data_unsigned_registers[i],calib_unsigned_registers[i]);
	calib_data[0]=0;calib_data[1]=0;
	}

	// For loop to get the signed data dig_T2-T3 and dig_P[2-9].
	for(int i=0;i<10;i++){
	i2c_burst_read(dev_i2c,BMP280_ADDR,calib_signed_registers[i],&calib_data[0],2);
	calib_data_signed_registers[i] = ((int16_t)((int16_t)(calib_data[1]))<<8 | ((int16_t)(calib_data[0])));
	printk("%d\t %d\t %d\t %d\t %x\n",i,calib_data[0],calib_data[1],calib_data_signed_registers[i],calib_signed_registers[i]);
	calib_data[0]=0;calib_data[1]=0;
	}
}



int32_t temperature_read_raw_data(void){
	int32_t rawdata;
	i2c_burst_read(dev_i2c,BMP280_ADDR,0xFA,&temp_data[0],3);		// Burst read the temperature data.
	rawdata = (int32_t) ((((uint32_t) (temp_data[0])) << 12) | (((uint32_t) (temp_data[1])) << 4) | ((uint32_t) temp_data[2] >> 4));
	// return uint32_t(temp_data[0])<<16 | uint32_t(temp_data[1])<<8 | uint32_t(temp_data[2]);
	return rawdata;		
}
float read_temperature(void){
    i2c_reg_write_byte(dev_i2c,BMP280_ADDR,REG_CTRL_MEAS,START_READING_DATA);

	int32_t var1, var2;

	int32_t temp = temperature_read_raw_data();

	var1 = ((((temp >> 3) - ((int32_t)calib_data_unsigned_registers[0] << 1))) * ((int32_t) calib_data_signed_registers[0])) >>11;

	var2 = (((((temp >> 4) - ((int32_t) calib_data_unsigned_registers[0])) * ((temp >> 4) - ((int32_t)calib_data_unsigned_registers[0]))) >> 12) * ((int32_t) calib_data_signed_registers[1])) >> 14;
	t_fine = var1+var2;

	float T = (t_fine  * 5 + 128) >>8;
	return T/100;
}


/**
*	FUNCTION: To get the raw data of pressure.
*/
int32_t pressure_read_raw_data(void){
	int32_t rawdata;
	i2c_burst_read(dev_i2c, BMP280_ADDR, 0xF7, &press_data[0],3);
	rawdata = (int32_t) ((((uint32_t) (press_data[0])) << 12) | (((uint32_t) (press_data[1])) << 4) | ((uint32_t) press_data[2] >> 4));
	return rawdata;
}

/**
* FUNCTION: To convert the raw data into human readable format. Adapted from the adafruit library.
*/

float read_pressure(void){
	int64_t var1, var2, p;
	read_temperature();																			// Required to get the t_fine variable.
	int32_t press = pressure_read_raw_data();

	var1 = ((int64_t)t_fine) - 128000;

	var2 = var1 * var1 *(int64_t) calib_data_signed_registers[6];
	var2 = var2 + ((var1 * (int64_t) calib_data_signed_registers[5]) << 17);
	var2 = var2 + (((int64_t) calib_data_signed_registers[4]) << 35);
	var1 = ((var1 * var1 * (int64_t)calib_data_signed_registers[3]) >> 8) + 
					((var1 * (int64_t)calib_data_signed_registers[2]) << 12);

	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)calib_data_unsigned_registers[1]) >> 33;

	if(var1 == 0)
		return 0;

	p = 1048576 - press;
	p = (((p<<31) - var2) * 3125) / var1;
	var1 = (((int64_t)calib_data_signed_registers[9]) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)calib_data_signed_registers[8]) * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (((int64_t) calib_data_signed_registers[7]) << 4);
	// printk("Value of p: %lld\n",p);
	return (float)p/256;
}

void bmp_read_press_temp_data(bmp280_t* bmp280_sensor_data_temp){
    bmp280_sensor_data_temp->temperature = read_temperature();
    bmp280_sensor_data_temp->pressure = read_pressure();
    return;
}

void print_data_bmp(bmp280_t* bmp280_sensor_data_temp){
    printk("Tempreature data: %f\t Pressure: %f\n",bmp280_sensor_data_temp->temperature,bmp280_sensor_data_temp->pressure); 
}