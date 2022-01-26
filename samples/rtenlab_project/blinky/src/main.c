/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uart_i2c.h"
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <stdio.h>
#include <drivers/gpio.h>


/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */
#define LED0_NODE DT_ALIAS(led0)
#define LED1_NODE DT_ALIAS(led2)

#if DT_NODE_HAS_STATUS(LED0_NODE, okay)
#define LED0	DT_GPIO_LABEL(LED0_NODE, gpios)
#define LED0_PIN		DT_GPIO_PIN(LED0_NODE, gpios)
#define LED0_FLAGS	DT_GPIO_FLAGS(LED0_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led0 devicetree alias is not defined"
#define LED0	""
#define LED0_PIN	0
#define LED0_FLAGS	0
#endif

#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
#define LED1	DT_GPIO_LABEL(LED1_NODE, gpios)
#define LED1_PIN		DT_GPIO_PIN(LED1_NODE, gpios)
#define LED1_FLAGS	DT_GPIO_FLAGS(LED1_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led1 devicetree alias is not defined"
#define LED1	""
#define LED1_PIN	0
#define LED1_FLAGS	0
#endif
#define LOW 0
#define HIGH 1

const struct device *dev0, *dev1;


uint8_t read_bit(void){
	uint8_t r;
	gpio_pin_set(dev1,LED1_PIN,LOW);
	k_usleep(3);
	gpio_pin_set(dev1,LED1_PIN,HIGH);
	k_usleep(10);
	r = gpio_pin_get(dev1,LED1_PIN);
	k_usleep(53);
	return r;
}

void write_bit(uint8_t v){
	if(v&1){
		gpio_pin_set(dev1,LED1_PIN,LOW);
		k_usleep(10);
		gpio_pin_set(dev1,LED1_PIN,HIGH);
		k_usleep(55);
	}
	else{
		gpio_pin_set(dev1,LED1_PIN,LOW);
		k_usleep(65);
		gpio_pin_set(dev1,LED1_PIN,HIGH);
		k_usleep(5);
	}
}

void write(uint8_t v){
	uint8_t bitMask;
	for(bitMask = 0x01; bitMask; bitMask<<=1){
		write_bit((bitMask&v)?1:0);
	}
	return;
}

uint8_t read(void){
	uint8_t bitMask;
	uint8_t r = 0;

	for(bitMask=0x01; bitMask;bitMask<<=1){
		if(read_bit())
			r|=bitMask;
	}
	return r;
}
void main(void)
{
	int ret;
	enable_uart_console();

	dev0 = device_get_binding(LED0);
	if (dev0 == NULL) {
		return;
	}
	dev1 = device_get_binding(LED1);
	if (dev1 == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev0, LED0_PIN, GPIO_OUTPUT_ACTIVE | LED0_FLAGS);
	if (ret < 0) {
		return;
	}

	ret = gpio_pin_configure(dev1, LED1_PIN, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_HIGH);
	if (ret < 0) {
		return;
	}
	
	write(0x33);
	uint8_t retu = read();
	printf("The Rom address is as follows: %d\n", retu);
	// while (1) {

	// 	// gpio_pin_set(dev0, LED0_PIN, (int)led_is_on);
	// 	// led_is_on = !led_is_on;
	// 	// k_usleep(1000000);
	// 	// gpio_pin_set(dev0, LED0_PIN, (int)led_is_on);
	// 	// k_usleep(1000000);
	// 	// printk("Dev is a great person...\n");
	// 	// while((gpio_pin_get(dev1,LED1_PIN))<=0){
	// 	// 	val = gpio_pin_get(dev1,LED1_PIN);
	// 	// printk("waiting for a rise with value of val: %d\n", val);
	// 	// k_usleep(1000);
	// 	// }
	// 	// val = gpio_pin_get(dev1,LED1_PIN);
	// 	// printk("The value at D5 is: %d\n", val);
	// }
}
