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
#include "sht/sht31.h"
#include "apds/apds9960.h"
#include "bmp/bmp280.h"
#include "lsm/lsm6ds33.h"
#include "scd/scd41.h"
#include "ds/Onewire.h"
#include "ds/Dallas_temperature.h"

#define SHT31
#define APDS9960
#define BMP280
#define LSM6DS33
#define SCD41
#define DS18B20

// Defines to get the format of the data sent using custom characteristics UUID
#define CPF_FORMAT_UINT8 	0x04
#define CPF_FORMAT_UINT16 	0x06

// Defines to get the unitsof the data sent using custom characteristics UUID

#define CPF_UNIT_NO_UNIT 	0x2700
#define CPF_UNIT_METER 		0x2701
#define CPF_UNIT_ACCEL 		0x2713
#define CPF_UNIT_ANG_VEL	0x2743

#define MY_STACK_SIZE 1500
#define MY_PRIORITY_1 5
#define MY_PRIORITY_2 3
#define MY_PRIORITY_3 2
#define THREAD_COUNT 3

#ifdef SHT31

// @brief 57812a99-9146-4e72-a4b7-5159632dee90
static struct bt_uuid_128 sht_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0x57812a99, 0x9146, 0x4e72, 0Xa4b7,  0X5159632dee90));

#endif

static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    notif_enabled ? "enabled" : "disabled";

	// LOG_INF("HRS notifications %s", notif_enabled ? "enabled" : "disabled");
}


BT_GATT_SERVICE_DEFINE(ess_svc,
// Primary Service for SHT Sensor. Basically Tempearture and sensor value.
#ifdef SHT31
	BT_GATT_PRIMARY_SERVICE(&sht_uuid),
// Caharactersitic temp. value. attrs[1]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

// charactersitic humidity value. attrs[4]
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),

	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif
);

#ifdef SHT31
/**
 * @brief  To notify the temperature and humidity value from the sht sensor to ble.
 * @note   Same can be implemented for different sensors. Not sure why humidity value has attrs[4].
 * @retval None
 */
void sht_notify(void)
{
	static sht31_t sensor_value;
    int16_t temp_value, hum_value;
	read_temp_hum(&sensor_value);
    temp_value = (uint16_t)((sensor_value.temp)*100);
	hum_value = (uint16_t)((sensor_value.humidity)*100);
    bt_gatt_notify(NULL, &ess_svc.attrs[1], &temp_value, sizeof(temp_value));
	bt_gatt_notify(NULL, &ess_svc.attrs[4], &hum_value, sizeof(hum_value));
}
#endif



struct k_mutex my_mutex;
void my_entry_point(void* a, void* b, void* c){
	ARG_UNUSED(a);
	ARG_UNUSED(b);
	ARG_UNUSED(c);
	k_tid_t some=k_current_get();
	int prio = k_thread_priority_get(some);
	while(1){
		if(k_mutex_lock(&my_mutex, K_FOREVER)!=0)
			printk("Unable to get the mutex lock for thread with prio: %d\n",prio);
		printk("Hello from the only thread %d!!!\n", prio);
		delay(1000);
		k_mutex_unlock(&my_mutex);
	}
}
K_THREAD_STACK_DEFINE(my_stack_area1, MY_STACK_SIZE);
K_THREAD_STACK_DEFINE(my_stack_area2, MY_STACK_SIZE);
// K_THREAD_STACK_DEFINE(my_stack_area3, MY_STACK_SIZE);
struct k_thread my_thread_data[THREAD_COUNT];

#ifdef APDS9960

/**
 * @brief  Notify the client the change in the proximity and ALS sensor value.
 */
void apds9960_notify(void)
{
	static apds9960_t sensor_value;
	int16_t clear;
	read_als_data(&sensor_value);
	clear = sensor_value.clear;
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[8], &sensor_value.clear, sizeof(sensor_value.clear));
	k_sleep(K_MSEC(500));
	printk("APDS Notified!!!\n");
	return;
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



void main(void){
	// enable_uart_console();
	configure_device();
	    int err;
	enable_uart_console();


	k_mutex_init(&my_mutex);	
	k_tid_t my_tid = k_thread_create(&my_thread_data[0], my_stack_area1, 
											K_THREAD_STACK_SIZEOF(my_stack_area1),
											my_entry_point, 
											NULL, NULL, NULL,
											MY_PRIORITY_1, 0, K_NO_WAIT);
	printk("TID for 1st thread: %d\n", my_tid);

	my_tid = k_thread_create(&my_thread_data[1], my_stack_area2, 
											K_THREAD_STACK_SIZEOF(my_stack_area2),
											my_entry_point, 
											NULL, NULL, NULL,
											MY_PRIORITY_2, 0, K_NO_WAIT);
	printk("TID for 2nd thread: %d\n", my_tid);
	// my_tid = k_thread_create(&my_thread_data[2], my_stack_area3, 
	// 										K_THREAD_STACK_SIZEOF(my_stack_area3),
	// 										my_entry_point, 
	// 										NULL, NULL, NULL,
	// 										MY_PRIORITY_3, 0, K_NO_WAIT);
	// printk("TID for 3rd thread: %d\n", my_tid);
	
}