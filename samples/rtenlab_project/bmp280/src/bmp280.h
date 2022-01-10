#ifndef BMP280_H
#define BMP280_H

#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include "uart_i2c.h"
#include <stdlib.h>
#include <kernel.h>


#define I2C_DEV "I2C_0"

#define BMP280_ADDR 0x77
#define REG_SENSOR_ID 0x58
#define REG_ID 0xD0
#define REG_RESET 0xE0
#define STATUS_REG_BMP 0xF3
#define REG_CTRL_MEAS 0xF4
#define REG_CONFIG 0xF5
#define REG_PRESSURE_MSB 0xF7
    #define REG_PRESSURE_LSB 0xF8
    #define REG_PRESSURE_XLSB 0xF9  // (bit 7,6,5,4)
#define REG_TEMP_MSB 0xFA
    #define REG_TEMP_LSB 0xFB
    #define REG_TEMP_XLSB 0xFC  // (bit 7,6,5,4)
/**
* [7:5] READING TEMP. SAMPLING X16
* [4:2] READING PRESS SAMPLING X16
* [1:0] SENSOR POWER MODE- FORCED MODE.
*/
#define START_READING_DATA 0XB6     //10110110. 

typedef struct{
    float temperature;
    float pressure;
}bmp280_t;


/**
* Checks the Sensor ID and make sure the sensor is BMP280.
*
* @returns:(uint8_t): 0, if the sensor is correct, 1 otherwise.
*/
uint8_t check_sensor(void);


/**
* Read the calibration register values from the sensor. Values remains constant.
*/
void read_calibration_registers(void);

/**
* Helper function to get the RAW data of the temperature sensor directly from the registers.
* @returns: The raw temperature in signed 32 bit-integer.
*/
int32_t temperature_read_raw_data(void);

/**
* Reads the temperature data and converts the data into human-readable form.
* @returns: Float value of the human-readable format. 
*/
float read_temperature(void);

/**
* Reads the pressure raw data from the registers. 
* @returns: Signed integer number to get the pressure value directly from the register.
*/
int32_t pressure_read_raw_data(void);

/**
* Reads the pressure data and converts it to human readable form.
* @returns: Float value of the human-readable format. 
*/
float read_pressure(void); 



#endif