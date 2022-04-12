/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include<stdio.h>
#include <zephyr.h>
#include <sys/printk.h>
// #include "uart_i2c.h"

#include<logging/log.h>
#include "uart_i2c.h"
#define LOG_MODULE_NAME something
#define LOG_LEVEL LOG_LEVEL_DBG

LOG_MODULE_REGISTER();

void main(void)
{
	enable_uart_console();
	LOG_DBG("verbose debug %d", 3);
	LOG_INF("everything is fine, mask=0x%x", 0xa1);
	LOG_WRN("warning: %s was seen", "something bad");
	LOG_ERR("error %d", 3);
}
