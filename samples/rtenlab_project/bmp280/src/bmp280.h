#ifndef BMP280_H
#define BMP280_H

#include <zephyr.h>
#include <sys/printk.h>


#define I2C_DEV "I2C_0"
#define mask(x) (1<<x)	

#define ID 0xD0
#define RESET 0xE0
#define STATUS_REG_BMP 0xF3
#define CTRL_MEAS 0xF4
#define CONFIG 0xF5
#define PRESSURE_MSB 0xF7
    #define PRESSURE_LSB 0xF8
    #define PRESSURE_XLSB 0xF9  // (bit 7,6,5,4)
#define TEMP_MSB 0xFA
    #define TEMP_LSB 0xFB
    #define TEMP_XLSB 0xFC  // (bit 7,6,5,4)

#endif