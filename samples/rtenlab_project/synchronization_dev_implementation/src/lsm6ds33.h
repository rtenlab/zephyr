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
#define delay(x) k_sleep(K_MSEC(x))

#define ACC_104_HZ_4G 0x28		// Set gyroscope data rate to 104HZ
#define GYRO_104_HZ_1000dps 0x28	// Set Accel. data rate to 104HZ
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

void enable_uart_console(void);
void configure_device(void);
void check_device(void);
void set_acc_data_rate_range(void);
void set_gyro_data_rate_range(void);
void status_reg(void);
void read_burst_data(lsm6ds33_t*);
void print_data(lsm6ds33_t*);
void lsm6ds33_init(void);
#endif