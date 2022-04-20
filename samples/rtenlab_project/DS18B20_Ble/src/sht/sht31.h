#ifndef SHT31_H
#define SHT31_H

#include<stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include "../uart_i2c.h"


	
#define mask(x) (1<<x)																				// Macro for masking bits.


#define STATUS_REG 0xF32D
#define ENABLE_HEATER 0x306D
#define SOFT_RESET 0x30A2
#define HIGH_MEAS_CLOCK_ENABLE 0x2C06
#define MED_MEAS_CLOCK_ENABLE 0x2C0D
#define LOW_MEAS_CLOCK_ENABLE 0x2C10
#define HIGH_MEAS_CLOCK_DIS 0X2400
#define MED_MEAS_CLOCK_DIS 0x240B
#define LOW_MEAS_CLOCK_DIS 0x2416



// Struct to hold the integer and fractional information of temperature and humidity.
typedef struct temphum{
	float temp;
	float humidity;
}sht31_t;


void enable_uart_console(void);
void configure_device(void);
void write_command(uint16_t);
uint16_t read_status(void);
void read_temp_hum(sht31_t*);
void print_data_sht(sht31_t*);

#endif