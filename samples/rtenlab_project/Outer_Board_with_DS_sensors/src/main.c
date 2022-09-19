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
#include "battery/battery.h"
#include "blinky/blink.c"

#define DS18B20
#define NUM_SENSORS 9
#define BATTERY
// #define MAIN_DEBUG
volatile bool BLE_CONNECTED;
#ifdef DS18B20
//@brief  UUID for clear_als apds sensor data: e66e54fc-4231-41ae-9663-b43f50cfcb3b
static struct bt_uuid_128 ds18b_primary_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xe66e54fc, 0x4231, 0x41ae, 0x9663, 0xb43f50cfcb3b));
#endif

#ifdef BATTERY
//@brief  UUID for battery data: b9ad8153-8145-4575-9d1a-ab745b5b2d08
static struct bt_uuid_128 battery_primary_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xb9ad8153, 0x8145, 0x4575, 0x9d1a, 0xab745b5b2d08));

// @brief Secondary UUID for battery data:0482d82-3a6f-4f52-b35f-c86eda8747fd
static struct bt_uuid_128 battery_secondary_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xe0482d82,0x3a6f,0x4f52,0xb35f,0xc86eda8747fd));
#endif

volatile bool notif_enabled;
static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    notif_enabled ? "enabled" : "disabled";

	// LOG_INF("HRS notifications %s", notif_enabled ? "enabled" : "disabled");
}

#define CPF_FORMAT_UINT16 	0x06
#define CPF_UNIT_NO_UNIT 	0x2700
#define CPF_VOLTAGE_UNIT	0x2B18
static const struct bt_gatt_cpf battery = {
	.format = CPF_FORMAT_UINT16,
	.unit = CPF_VOLTAGE_UNIT,
};

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

	// Caharactersitic Co2 value. attrs[16]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	// Caharactersitic Co2 value. attrs[19]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

	// Caharactersitic Co2 value. attrs[22]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

		// Caharactersitic Co2 value. attrs[25]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

#endif
#ifdef BATTERY
	BT_GATT_PRIMARY_SERVICE(&battery_primary_uuid),
	// Characteristic for Battery Voltage data. attrs[29]
	BT_GATT_CHARACTERISTIC(&battery_secondary_uuid.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&battery),
#endif
);

#ifdef DS18B20
void ds18b_notify(void){
	// This gives the number of DS18B20 devices we have. Hardcoded because something behaves spuriously.
	volatile uint8_t n_devices = NUM_SENSORS;//getDeviceCount();
	// Helper variable.
	static float sensor_value;
	// DS to get the 64-bit address of each device.
	uint8_t address[8];
	// Start collecting temperature from all the sensors.
	requestTemperatures();
	// DS to hold the data for all the devices and send it over to BLE.
	static uint32_t some[NUM_SENSORS];
	// Loop to get the temperature from all the individual sensors.
	for(int i=0; i<n_devices; i++){
		// Get the address of each sensor.
		if(getAddress(address, i)==false){
			printk("Problem with get Address call\n");
		}
		// Get the temperature of each sensor on at a time. Referenced by relative numbering.
		sensor_value = getTempCByIndex(i);
		// Change the float value to 2 decimal integer.
		some[i] = (uint16_t)((sensor_value)*100);
		// Debug message.
		printk("Before: some[%d]: %d\n", i, some[i]);
		// Make space to store the 2 bytes of unique sensor address and store the last 2 bytes as the LSB.
		some[i] = (some[i]<<8) | address[7];
		// Debug message
		printk("Address: 0x%x\n", address[7]);
		delay(250);
	}
	printk("Done with DS Reading\n");
	uint8_t index=1;
	
	for(int i=0; i<=NUM_SENSORS; i++){

		index = 1+i*3;
		printk("Index: %d\n", index);
		bt_gatt_notify(NULL, &ess_svc.attrs[index], &some[i], sizeof(some[0]));
		k_sleep(K_MSEC(500));
	}
}
#endif

#ifdef BATTERY
void batt_notify(){
	float batt = battery_sample();

	if(batt < 0)
		printk("failed to read battery voltage: %d", (int)batt);
	
	batt *= 100;
	uint16_t batt_int = (uint16_t)(batt);
	printk("This is what I am sharing currently: %d\n", batt_int);
	bt_gatt_notify(NULL, &ess_svc.attrs[29],&batt_int, sizeof(batt_int));
	return;
}
#endif

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	// BT_DATA_BYTES(BT_DATA_GAP_APPEARANCE, 0x56, 0x05, 0x16),
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
	led_on_blink1(false);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	int err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if (err) {
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}
	printk("Disconnected (reason 0x%02x)\n", reason);
	led_on_blink1(true);
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
	led_on_blink1(true);
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
	printk("BAck with bt_ready!!!\n");
	bt_conn_cb_register(&conn_callbacks);

#ifdef DS18B20
extern const struct device *dev_ds18b20;
	int ret;
	dev_ds18b20 = device_get_binding(DS_SENSOR);
	if (dev_ds18b20 == NULL) {
		printk("Couldn't get binding for some reason!!!\n");
		return;
	}

	ret = gpio_pin_configure(dev_ds18b20, DS_SENSOR_PIN, GPIO_OUTPUT);
	if (ret < 0) {
		return;
	}
	DallasTemperature_begin();
	printk("Waiting for notifications enabled!!!\n");
#endif
    while (1) {
		// printk("Sending data\n");
		if(notif_enabled){
			led_on_blink0(true);			
			k_sleep(K_SECONDS(5));
		#ifdef BATTERY
			batt_notify();
		#endif
		#ifdef DS18B20
			ds18b_notify();
		#endif
			led_on_blink0(false);
		#ifdef MAIN_DEBUG
			k_sleep(K_SECONDS(70));
		#else
			k_sleep(K_MINUTES(20));
		#endif
		}
    }
}