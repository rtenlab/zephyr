#include "uart_i2c.h"
#define I2C_DEV "I2C_0"


/**
*@FUNCTION: API to configure the uart console for printk statements.
*/
void enable_uart_console(void){
	const struct device  *dev_usb;																										// Device for USB Console.
	uint32_t dtr=0;
	

	dev_usb = device_get_binding(CONFIG_UART_CONSOLE_ON_DEV_NAME);
	if(usb_enable(NULL))
		return;
	
	while(!dtr)
		uart_line_ctrl_get(dev_usb,UART_LINE_CTRL_DTR, &dtr);	
	  
	return;
}

/**
*@FUNCTION: API to configure the I2C device.
*/
void configure_device(void){
	dev_i2c = device_get_binding(I2C_DEV);
	i2c_configure(dev_i2c, I2C_SPEED_SET(I2C_SPEED_STANDARD));
	if (dev_i2c == NULL) {
		printk("I2C: Device driver not found\n");
	}
	else{
		printk("I2C: Device driver found!!!\n");
	}
	return;
}