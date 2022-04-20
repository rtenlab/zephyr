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
#include <usb/usb_device.h>

#include "uart_i2c.h"
#include "ds/Onewire.h"
#include "ds/Dallas_temperature.h"
#include "sht/sht31.h"

#define DS18B20

volatile bool BLE_CONNECTED;
#ifdef DS18B20
//@brief  UUID for clear_als apds sensor data: e66e54fc-4231-41ae-9663-b43f50cfcb3b
static struct bt_uuid_128 ds18b_primary_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xe66e54fc, 0x4231, 0x41ae, 0x9663, 0xb43f50cfcb3b));
#endif

volatile bool notif_enabled;
static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    notif_enabled ? "enabled" : "disabled";

	// LOG_INF("HRS notifications %s", notif_enabled ? "enabled" : "disabled");
}


BT_GATT_SERVICE_DEFINE(ess_svc,

#ifdef DS18B20
	BT_GATT_PRIMARY_SERVICE(&ds18b_primary_uuid),
// Caharactersitic Co2 value. attrs[1]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	// Caharactersitic Co2 value. attrs[4]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
			
	// Caharactersitic Co2 value. attrs[7]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	
	// Caharactersitic Co2 value. attrs[10]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	
	// Caharactersitic Co2 value. attrs[13]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif
);

#ifdef DS18B20
void ds18b_notify(void){
	volatile uint8_t n_devices = getDeviceCount();
	static float sensor_value;
	requestTemperatures();
	static uint16_t some[5];
	for(int i=0; i<n_devices; i++){
		sensor_value = getTempCByIndex(i);
		some[i] = (uint16_t)((sensor_value)*100);
		printk("Temp[%d]: %d\n", i,some[i]);
		delay(10);
	}
	printk("Done with DS Reading\n");
	bt_gatt_notify(NULL, &ess_svc.attrs[1], &some[0], sizeof(some[0]));
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[4], &some[1], sizeof(some[0]));
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[7], &some[2], sizeof(some[0]));
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[10], &some[3], sizeof(some[0]));
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[13], &some[4], sizeof(some[0]));

}
#endif

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0x00, 0x03),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_ESS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
	BLE_CONNECTED=true;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
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

	// enable_uart_console();
	// configure_device();
    int err;
	
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	bt_conn_cb_register(&conn_callbacks);

#ifdef DS18B20
extern const struct device *dev_ds18b20;
	int ret;
	dev_ds18b20 = device_get_binding(LED1);
	if (dev_ds18b20 == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev_ds18b20, LED1_PIN, GPIO_OUTPUT);
	if (ret < 0) {
		return;
	}
	DallasTemperature_begin();
#endif
    while (1) {
		// printk("Sending data\n");
		if(notif_enabled){
			k_sleep(K_SECONDS(5));
			#ifdef DS18B20
					ds18b_notify();
			#endif
			k_sleep(K_SECONDS(3));
		}
    }
}