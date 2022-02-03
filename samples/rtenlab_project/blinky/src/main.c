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

#define LED1_NODE DT_ALIAS(led2)

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

static inline void udelay(int secs){
	k_busy_wait(secs);
}

const struct device *dev0, *dev1;

uint8_t reset(void){
	int key = irq_lock();
	uint8_t r;
	gpio_pin_configure(dev1, LED1_PIN, GPIO_OUTPUT);
	gpio_pin_set(dev1,LED1_PIN,LOW);
	udelay(480);
	gpio_pin_set(dev1,LED1_PIN,HIGH);
	udelay(70);
	// while((gpio_pin_get(dev1,LED1_PIN))!=0){
	// 	r = gpio_pin_get(dev1,LED1_PIN);
	// 	printk("stuck in teh while loopwith value of r: %d\n",r);
	// }
	gpio_pin_configure(dev1, LED1_PIN, GPIO_INPUT);
	r = gpio_pin_get(dev1,LED1_PIN);
	udelay(410);
	irq_unlock(key);
	return !r;
}

// uint8_t read_bit(void){
// 	uint8_t r;
// 	gpio_pin_configure(dev1, LED1_PIN, GPIO_OUTPUT);
// 	gpio_pin_set(dev1,LED1_PIN,LOW);
// 	udelay(3);
// 	gpio_pin_set_raw(dev1,LED1_PIN,HIGH);
// 	udelay(13);
// 	gpio_pin_configure(dev1, LED1_PIN, GPIO_INPUT);
// 	r = gpio_pin_get(dev1,LED1_PIN);
// 	udelay(53);
// 	return r;
// }

// uint8_t read(void){
// 	uint8_t bitMask;
// 	uint8_t r = 0;

// 	for(bitMask=0x01; bitMask;bitMask<<=1){
// 		if(read_bit())
// 			r|=bitMask;
// 	}
// 	return r;
// }

void write_bit(uint8_t v){
	// printk("Outputting bit: %d\n",v);
	if(v&1){
		// int key = irq_lock();
		gpio_pin_configure(dev1, LED1_PIN, GPIO_OUTPUT);
		gpio_pin_set(dev1,LED1_PIN,LOW);
		udelay(10);
		gpio_pin_set(dev1,LED1_PIN,HIGH);
		// irq_unlock(key);
		udelay(55);
	}
	else{
		// int key = irq_lock();
		gpio_pin_configure(dev1, LED1_PIN, GPIO_OUTPUT);
		gpio_pin_set(dev1,LED1_PIN,LOW);
		udelay(65);
		gpio_pin_set(dev1,LED1_PIN,HIGH);
		// irq_unlock(key);
		udelay(5);
	}
	return;	
}

void write(uint8_t v){
	uint8_t bitMask;
	for(bitMask = 0x01; bitMask; bitMask<<=1){
		write_bit((bitMask&v)?1:0);
	}
	return;
}


void main(void)
{
	int ret;
	// enable_uart_console();

	dev1 = device_get_binding(LED1);
	if (dev1 == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev1, LED1_PIN, GPIO_OUTPUT);
	if (ret < 0) {
		return;
	}
	
	while(1){
	reset();
	// write(0x33);
	// gpio_pin_set(dev1, LED1_PIN, LOW);
	// delay(100);
	// write(0x10);
	// gpio_pin_set(dev1,LED1_PIN, HIGH);
	// delay(100);
	}
	// write(0xFF);
	
	// uint8_t retu = read();
	// printf("The Rom address is as follows: 0x%x\n", retu);
	// retu=read();
	// printf("The Rom address is as follows: 0x%x\n", retu);
}
