#ifndef SCD41_H
#define SCD41_H

#include<stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include "uart_i2c.h"

#define CRC8_POLYNOMIAL 0x31
#define CRC8_INIT 0xFF
#define NUM_BYTES_TO_SENSOR 3
#define READ_MEASUREMENT 0xec05
#define GET_SERIAL_NUMBER 0x3682
#define MEASURE_SINGLE_SHOT 0x219d


// data structure to get the data from the sensor.										
typedef struct data{
	uint16_t Co2;
	float temp, hum;
}scd41_t;


// defining the functions used in the main.c file.
uint8_t generate_crc(const uint8_t*, uint16_t);
void enable_uart_console(void);
void configure_device(void);
void get_formatted_command(uint16_t);
void get_serial_number(void);
void read_sensor_data(uint8_t*, int);
void measure_single_shot(scd41_t*);
void print_data(scd41_t*);
void enable_uart_console(void);
void configure_device(void);
#endif