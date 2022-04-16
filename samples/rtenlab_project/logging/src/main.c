/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include<stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
// #include "uart_i2c.h"

#include "uart_i2c.h"
#include "sht31.h"



void my_work_handler(struct k_work *work){
	sht31_t sht31_data;
	printk("Requesting SCD41 data\n");
	read_temp_hum(&sht31_data);
	print_data_sht(&sht31_data);
	printk("Data printed. Now retiring.\n");
}

K_WORK_DEFINE(my_work, my_work_handler);

void my_timer_handler(struct k_timer* dummy){
	k_work_submit(&my_work);
}
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

void main(void)
{
	enable_uart_console();
	configure_device();
	k_timer_start(&my_timer, K_SECONDS(30), K_SECONDS(30));

}
