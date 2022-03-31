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
// Select mode of the sensor. 1 if forced mode and 0 if sleep mode.
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
#define BME680_Res_Heat0 0x5A
#define BME680_Gas_Wait0 0x64
// LSB nibble 0000 for nb_conv->0 and run_gas->1; to start the gas conversion.
#define BME680_Gas_Wait0_msk 0x3F

#define BME680_par_g1 0xED
#define BME680_par_g2_LSB 0xEB
#define BME680_par_g2_MSB 0xEC
#define BME680_par_g3 0xEE
#define BME680_res_heat_range 0x02
#define BME680_res_heat_range_mask 0x30
#define BME680_res_heat_val 0x00
#define BME680_range_sw_err 0x04
#define BME680_RSERROR_MSK 0xF0

#define FPU_EN

typedef struct {
    uint8_t amb_temp;
    int8_t para1;
    int16_t para2;
    int8_t para3;
    /*! Variable to store heater resistance range */
	uint8_t res_heat_range;
	/*! Variable to store heater resistance value */
	int8_t res_heat_val;
    int8_t range_sw_err;
}bme680_gas_par_t;

typedef struct{
    uint8_t gas_range,
            gas_valid_bit,
            heater_stability_bit;
    uint16_t adc_gas_res;
}bme680_gas_data_t;

bool bme680_getid(void);
bool bme680_get_gas_calib_data(bme680_gas_par_t*);
bool bme680_set_power_mode(uint8_t);
bool bme680_set_heater_conf(uint16_t , uint16_t , bme680_gas_par_t*);
#ifdef FPU_EN
float bme680_calculate_res_heat(uint16_t , bme680_gas_par_t*);
#else
uint8_t bme680_calculate_res_heat(uint16_t , bme680_gas_par_t*);
#endif

uint8_t bme680_calc_gas_wait(uint16_t);
#ifdef FPU_EN
float bme680_calc_gas_resistance(bme680_gas_data_t* , bme680_gas_par_t*);
#else
uint32_t bme680_calc_gas_resistance(bme680_gas_data_t* , bme680_gas_par_t*);
#endif

bool bme680_get_raw_gas_data(bme680_gas_data_t*);
bool bme680_check_new_data(void);






#endif