#ifndef UART_I2C_H
#define UART_I2C_H 

#include <device.h>
#include <drivers/i2c.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>

extern const struct device *dev_i2c;

void enable_uart_console(void);
void configure_device(void);

#endif