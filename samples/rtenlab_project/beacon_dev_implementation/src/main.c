/* main.c - Application main entry point */

/*
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/types.h>
#include <stddef.h>
#include <sys/printk.h>
#include <sys/util.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/gap.h>

#include "uart_i2c.h"
#include "sht31.h"

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)


/*
 * Set Advertisement data. Based on the Eddystone specification:
 * https://github.com/google/eddystone/blob/master/protocol-specification.md
 * https://github.com/google/eddystone/tree/master/eddystone-url
 */
static const struct bt_data ad[] = {
		BT_DATA_BYTES(BT_DATA_FLAGS, BT_LE_AD_NO_BREDR),
		// BT_DATA_BYTES(BT_DATA_UUID16_ALL, 0xaa, 0xfe),
		BT_DATA_BYTES(BT_DATA_TX_POWER),
		BT_DATA_BYTES(BT_GAP_DATA_LEN_MAX, 0xFF),
		BT_DATA_BYTES(BT_GAP_DATA_LEN_MAX, 0x33),
};

/* Set Scan Response data */
// static 
static sht31_t something;

void write(sht31_t* something_local){
	something.temp = something_local->temp;
	something.humidity = something_local->humidity;
	return;
}

static void bt_ready(int err)
{
	char addr_s[BT_ADDR_LE_STR_LEN];
	bt_addr_le_t addr = {0};
	size_t count = 1;

	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	struct bt_data sd[] = {
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
	BT_DATA_BYTES(BT_GAP_DATA_LEN_MAX, something.temp),
	BT_DATA_BYTES(BT_GAP_DATA_LEN_MAX, something.humidity),
};

	printk("Bluetooth initialized\n");

	/* Start advertising */
	/* change this to BT_LE_ADV_CONN to be able to establish connection. Need to write some additional code to actually connect to the data.*/
	err = bt_le_adv_start(BT_LE_ADV_NCONN_IDENTITY, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}


	/* For connectable advertising you would use
	 * bt_le_oob_get_local().  For non-connectable non-identity
	 * advertising an non-resolvable private address is used;
	 * there is no API to retrieve that.
	 */

	bt_id_get(&addr, &count);
	bt_addr_le_to_str(&addr, addr_s, sizeof(addr_s));

	printk("Beacon started, advertising as %s\n", addr_s);
}

void main(void)
{
	sht31_t sht31_sensor_data;
	int err;
	enable_uart_console();
	configure_device();
	printk("Starting Beacon Demo\n");

	/* Initialize the Bluetooth Subsystem */
	while(1){
		read_temp_hum(&sht31_sensor_data);
		write(&sht31_sensor_data);
		print_data_sht(&sht31_sensor_data);
		delay(100);
	err = bt_enable(bt_ready);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
	}
	}

	
}
