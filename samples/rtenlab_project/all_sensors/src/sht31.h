#include<stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
#include <device.h>
#include <drivers/i2c.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>


#define I2C_DEV "I2C_0"
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
#define delay(x) k_sleep(K_MSEC(x))


// Struct to hold the integer and fractional information of temperature and humidity.
typedef struct temperature{
	int32_t integer;
	uint8_t frac;	
}temperature_t;

typedef struct humidity{
	uint32_t integer;
	uint8_t frac;
}humidity_t;

typedef struct temphum{
	temperature_t temp;
	humidity_t humidity;
}temphum_t;


void enable_uart_console(void);
void configure_device(void);
void write_command(uint16_t);
uint16_t read_status(void);
void read_temp_hum(temphum_t*);
void print_data(temphum_t*);
