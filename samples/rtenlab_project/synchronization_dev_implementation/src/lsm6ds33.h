#ifndef LSM6DS33_H
#define LSM6DS33_H

#include <stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include "uart_i2c.h"

// Address of sensor on I2C bus.
#define LSM6DS_ADDR 0x6A
//  Device ID of the sensor.
#define DEV_ID 105
// Control register for the Accelerometer. Sets ODR and Bandwith. P.47 (9.11)
#define CTRL1_XL 0x10
// Control register for the Gyroscope. Sets ODR and Bandwaith. P.48 (9.12)
#define CTRL2_G 0x11
// Control register 3. Required for BDU. Not necessary to implement. P.48 (9.13)
#define CTRL3_C 0x12
// Control register 4. Set the 0th bit STOP_ON_FTH to stop the FIFO collection when the watermark flag rises. P.49 (9.14)
#define CTRL4_C 0x13
// Status Register. P.56 (9.24)
#define STATUS_REG_LSM 0x1E
// Output Low register for the temperature. Can burst read 14 uint8_t from this register to get all the data. P.57 (9.25)
#define OUT_TEMP_L 0x20
// Output High register for the temperature register.
#define OUT_TMP_H 0x21
// Required to set the high performance mode or low power mode on gyroscope and Acccel.
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


// Bit Mask to set the Low Power Mode for gyro.
#define GYRO_LOW_POWER_MODE 0x80
#define GYRO_HIGH_PERF_MODE 0x00

// Bit Mask to set the Low power mode for accel.
#define ACCEL_LOW_POWER_MODE 0x10
#define ACEEL_HIGH_PERF_MODE 0x00

// Current 26Hz
#define ACC_26_HZ_4G 0x28		// Set gyroscope data rate to 26Hz. Make sure to read the datasheet for this. Also the scale selection is 4G. 2-> 26Hz, 8->4G.
#define GYRO_26_HZ_1000dps 0x28	// Set Accel. data rate to 26Hz. Make sure to read the datasheet for this. Also the scale selection is 1000dps. 2-> 26Hz, 8->1000dps.
// below values can be found on this website: https://github.com/adafruit/Adafruit_Sensor/blob/master/Adafruit_Sensor.h
#define SENSORS_DPS_TO_RADS (0.017453293F)		
#define SENSORS_GRAVITY_STANDARD (9.80665f)


// Define to get the output in the FIFO Mode. Useful to detect vibrations.
#define FIFO_MODE
// Bit mask if the FIFO_FULL flag is set.
#define FIFO_FULL 0x2000
// Bit mask if the FIFO_EMPTY flag is set.
#define FIFO_EMPTY 0x1000

// Structure to hold the Gyroscope and Accelerometer data.
typedef struct lsm6ds33{
	uint16_t temp;
	float gyroX,
		  gyroY,
		  gyroZ, 
		  accelX,
		  accelY,
		  accelZ;
}lsm6ds33_t;

// Structure to hold the setting for FIFO.
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

// FIFO helper functions
/**
 * @brief  Set the current setting of the FIFO.
 * @note   
 * @param  setting: Global variable. Each parameter of the setting is changed inside this function.
 * @retval None
 */
void lsm6ds33_set_fifo_settings(fifo_t*);

/**
 * @brief  Begin the FIFO and call the @lsm6ds33_set_fifo_settings function call to set all the settings before calling this function.
 * @note   fifo_t* a FIFO setting struct reference.
 * @retval None
 */
void lsm6ds33_fifo_begin(fifo_t*);

/**
 * @brief  Get the current FIFO status. Reads FIFO_STATUS registers to get the data.
 * @note   
 * @retval Returns a 16bit value which possess data out of FIFO_STATUS1 & FIFO_STATUS2 register.
 */
uint16_t lsm6ds33_fifo_status(void);

/**
 * @brief  Read 2 bytes from the fifo. Gets Raw data out of the FIFO_DATA_OUT_L & FIFO_DATA_OUT_H registers.
 * @note   
 * @retval Returns a word with the current fifo data.
 */
uint16_t lsm6ds33_fifo_read(void);

/**
 * @brief  Clears the content of the FIFO. Reads the data until FIFO_EMPTY flag is set.
 * @note   
 * @retval None
 */
void lsm6ds33_fifo_clear(void);

/**
 * @brief  Processes the raw data and returns a human readable format of the gyroscope data.
 * @note   
 * @retval Float processed value of the raw gyroscope data.
 */
float lsm6ds33_fifo_get_gyro_data(uint16_t);

/**
 * @brief  Processes the raw data and returns a human readable format of the accelerometer data.
 * @note   
 * @retval Float Processed value of the raw accelerometer data.
 */
float lsm6ds33_fifo_get_accel_data(int16_t);

/**
 * @brief  Change the current mode of the data. 
 * @param uint8_t 0-> bypassmode; otherwise->FIFO_Mode
 * @note   
 * @retval None
 */
void lsm6ds33_fifo_change_mode(uint8_t);


#endif