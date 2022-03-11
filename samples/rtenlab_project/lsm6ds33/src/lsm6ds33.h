#ifndef LSM6DS33_H
#define LSM6DS33_H

#include <stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include "uart_i2c.h"


#define DEV_ID 105
#define CTRL1_XL 0x10
#define CTRL2_G 0x11
#define STATUS_REG_LSM 0x1E
#define OUT_TEMP_L 0x20
#define OUT_TMP_H 0x21
#define CTRL6_ACCEL 0x15
#define CTRL6_GYRO 0x16

// FIFO Control Registers. Section 9.2-9.6 Datasheet
#define FIFO_CTRL1 0x06 // SET THE THRESHOLD / WATERMARK FLAG
#define FIFO_CTRL2 0x07 // Higher bits for the watermark flag threshold.
#define FIFO_CTRL3 0x08 // Set teh decimation rate for Gyro and Accel. 000->Gyro sensor not on FIFO; 001->No Decimation
#define FIFO_CTRL4 0x09 // Not relevant. Leave as is.
// CTRL5L: Mode: 000->Bypass Mode; 001->FIFO mode; 110->Continuous Mode.
#define FIFO_CTRL5 0x0A // Set FiFO Mode [First 3 bits] and ODR [Next 4 bits.] [0DDDDMMM]
#define FIFO_STATUS1 0x3A // Number of unread words stored in FIFO
#define FIFO_STATUS2 0x3B // Number of unread words stored in FIFO + FTH + FIFO_OVER_RUN + FIFO_FULL
#define FIFO_DATA_OUT_L 0x3E // LSB of the output port of FIFO data
#define FIFO_DATA_OUT_H 0x3F // MSB of the output port of FIFO data



#define GYRO_LOW_POWER_MODE 0x80
#define GYRO_HIGH_PERF_MODE 0x00

#define ACCEL_LOW_POWER_MODE 0x10
#define ACEEL_HIGH_PERF_MODE 0x00
// Need to check if the below values are correct or not
#define ACC_26_HZ_4G 0x28		// Set gyroscope data rate to 26Hz. Make sure to read the datasheet for this. Also the scale selection is 4G. 2-> 26Hz, 8->4G.
#define GYRO_26_HZ_1000dps 0x28	// Set Accel. data rate to 26Hz. Make sure to read the datasheet for this. Also the scale selection is 1000dps. 2-> 26Hz, 8->1000dps.
// below values can be found on this website: https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h
#define SENSORS_DPS_TO_RADS (0.017453293F)		
#define SENSORS_GRAVITY_STANDARD (9.80665f)




typedef struct lsm6ds33{
	uint16_t temp;
	float gyroX,
		  gyroY,
		  gyroZ, 
		  accelX,
		  accelY,
		  accelZ;
}lsm6ds33_t;

typedef struct{
	uint16_t threshold;
	uint16_t sample_rate;
	uint16_t mode;	
	uint8_t gyro_decimation_rate;
	uint8_t axel_decimation_rate;
	
}fifo_t;

void gyro_set_power_mode(uint8_t);
void accel_set_power_mode(uint8_t);
void get_accel_rate(void);
void enable_uart_console(void);
void configure_device(void);
void check_device(void);
void set_acc_data_rate_range(void);
void set_gyro_data_rate_range(void);
void status_reg(void);
void read_burst_data(lsm6ds33_t*);
void print_data(lsm6ds33_t*);
void lsm6ds33_init(void);
void lsm6ds33_fifo_begin(fifo_t*);
uint16_t lsm6ds33_fifo_status(void);
uint16_t lsm6ds33_fifo_read(void);
void lsm6ds33_fifo_clear(void);

#endif