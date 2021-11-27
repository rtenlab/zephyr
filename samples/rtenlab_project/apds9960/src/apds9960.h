#ifndef _APDS9960_H
#define _APDS9960_H

#include <errno.h>
#include <zephyr.h>
#include <sys/printk.h>

#define mask(x) (1<<x)									// Macro for masking bits.
#define delay(x)    k_sleep(K_MSEC(x))

#define APDS_ADDR 0x39
#define APDS_PDATA 0x9c
#define APDS_CDATAL 0x94
#define APDS_CDATAH 0x95
#define APDS_RDATAL 0x96
#define APDS_RDATAH 0x97
#define APDS_GDATAL 0x98
#define APDS_GDATAH 0x99
#define APDS_BDATAL 0x9A
#define APDS_BDATAH 0x9B
#define APDS_ENABLE 0x80

typedef struct apds_data{
	uint8_t proximity;
	uint16_t clear;
	uint16_t red;
	uint16_t green;
	uint16_t blue;
}apds9960_t;

void read_proximity_data(apds9960_t*);
void read_als_data(apds9960_t*);
void enable_apds_sensor(void);
void print_data_apds(apds9960_t*);

#endif