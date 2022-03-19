#ifndef _BME680_H
#define _BME680_H

// Necessary includes.
#include <zephyr.h>
#include <sys/printk.h>
#include <stdio.h>
#include "uart_i2c.h"
#include <stdlib.h>
#include <kernel.h>


// Defines specific for the board.
#define I2C_DEV "I2C_0"
/**
* Note: Connect the SDO pin to GND to make the address 0x76. 
* Otherwise the address is 0x77 which is clashing with bme280 sensor.**/
#define BME680_Addr 0x76     

#define BME680_Status_Reg 0x73
#define BME680_Reset 0xE0
#define BME680_ID 0xD0
#define BME680_Config 0x75
#define BME680_Ctrl_meas 0x74
#define BME680_Ctrl_hum 0x72
#define BME680_Ctrl_gas_1 0x71
#define BME680_Ctrl_gas_0 0x70
#define BME680_Gas_r_lsb 0x2B
#define BME680_Gas_r_msb 0x2A
#define BME680_Hum_lsb 0x26
#define BME680_Hum_msb 0x25
#define BME680_Temp_xlsb 0x24
#define BME680_Temp_lsb 0x23
#define BME680_Temp_msb 0x22
#define BME680_Press_xlsb 0x21
#define BME680_Press_lsb 0x20
#define BME680_Press_msb 0x1F
#define BME680_Eas_status_0 0x1D


bool bme680_getid(void);







#endif