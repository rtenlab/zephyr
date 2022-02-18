/* main.c - Application main entry point */

/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdbool.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <errno.h>
#include <sys/printk.h>
#include <sys/byteorder.h>
#include <zephyr.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <bluetooth/services/bas.h>
#include <usb/usb_device.h>

#include "uart_i2c.h"
#include "sht31.h"

sht31_t sht31_var;



static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    notif_enabled ? "enabled" : "disabled";

	// LOG_INF("HRS notifications %s", notif_enabled ? "enabled" : "disabled");
}

static void hum_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    notif_enabled ? "enabled" : "disabled";

	// LOG_INF("HRS notifications %s", notif_enabled ? "enabled" : "disabled");
}

BT_GATT_SERVICE_DEFINE(ess_svc,
	BT_GATT_PRIMARY_SERVICE(BT_UUID_ESS),
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_NONE, NULL, NULL, NULL),
	BT_GATT_CCC(hum_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),					
	

	// BT_GATT_CHARACTERISTIC(BT_UUID_HRS_BODY_SENSOR, BT_GATT_CHRC_READ,
	// 		       HRS_GATT_PERM_DEFAULT & GATT_PERM_READ_MASK,
	// 		       read_blsc, NULL, NULL),
	// BT_GATT_CHARACTERISTIC(BT_UUID_HRS_CONTROL_POINT, BT_GATT_CHRC_WRITE,
	// 		       HRS_GATT_PERM_DEFAULT & GATT_PERM_WRITE_MASK,
	// 		       NULL, NULL, NULL),
);

void humidity_notify(void){
	sht31_t sensor_value;
	read_temp_hum(&sensor_value);
	int16_t hum_value = (uint16_t)((sensor_value.humidity)*100);
	bt_gatt_notify(NULL, &ess_svc.attrs[3], &hum_value, sizeof(hum_value));
}
void ess_notify(void)
{
    // u8_t temp[2];
    // if (!ess_do_notify) {
    //     return;
    // }
	static sht31_t sensor_value;
    // uint8_t temp;
    int16_t temp_value;
	read_temp_hum(&sensor_value);
    temp_value = (uint16_t)((sensor_value.humidity)*100);
	// hum_value = (uint16_t)((sensor_value.humidity)*100);
    // temp = (uint8_t)sensor_value.temp;
    // temp[0] = (uint8_t)((sensor_value.temp-temp[1])*100);
    // temp[0] = (uint8_t)((value     ) & 0xFF);
    // temp[1] = (uint8_t)((value >> 8) & 0xFF);
    bt_gatt_notify(NULL, &ess_svc.attrs[1], &temp_value, sizeof(temp_value));
	humidity_notify();
	// bt_gatt_notify(NULL, &ess_svc.attrs[3], &hum_value, sizeof(hum_value));
}

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0x00, 0x03),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_ESS_VAL),
		      BT_UUID_16_ENCODE(BT_UUID_BAS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason 0x%02x)\n", reason);
}

static struct bt_conn_cb conn_callbacks = {
	.connected = connected,
	.disconnected = disconnected,
};

static void bt_ready(void)
{
	int err;

	printk("Bluetooth initialized\n");

	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");
}


void main(void)
{
	configure_device();
    int err;
	// enable_uart_console();
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	bt_conn_cb_register(&conn_callbacks);
    
    // ess_init();
    
    while (1) {
        delay(1000);
        ess_notify();
    }
}