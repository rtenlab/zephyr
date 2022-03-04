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

#ifdef SHT31

// @brief 57812a99-9146-4e72-a4b7-5159632dee90
static struct bt_uuid_128 sht_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0x57812a99, 0x9146, 0x4e72, 0Xa4b7,  0X5159632dee90));

#endif

#ifdef APDS9960

// @brief  UUID for apds sensor data: ebcc60b7-974c-43e1-a973-426e79f9bc6c 
static struct bt_uuid_128 apds_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xebcc60b7, 0x974c, 0x43e1, 0Xa973,  0X426e79f9bc6c));


//  @brief  UUID for apds sensor data: 1441e94a-74bc-4412-b45b-f1d91487afe5
static struct bt_uuid_128 apds_prox_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0x1441e94a, 0x74bc, 0x4412, 0Xb45b,  0X4f1d91487afe5));


//  @brief  UUID for red_als apds sensor data: 3c321537-4b8e-4662-93f9-cb7df0e437c5
static struct bt_uuid_128 apds_als_red_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0x3c321537, 0x4b8e, 0x4662, 0x93f9, 0xcb7df0e437c5));


//  @brief  UUID for blue_als apds sensor data: 47024a73-790e-48ba-aac4-7d9e018572ba
static struct bt_uuid_128 apds_als_blue_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0x47024a73, 0x790e, 0x48ba, 0xaac4, 0x7d9e018572ba));

// 	@brief  UUID for green_als apds sensor data: 2f15eb47-9512-4ce3-8897-2f4460df7be4
static struct bt_uuid_128 apds_als_green_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0x2f15eb47, 0x9512, 0x4ce3, 0x8897, 0x2f4460df7be4));

// 	@brief  UUID for clear_als apds sensor data: e960c9b7-e0ed-441e-b22c-d93252fa0fc6

static struct bt_uuid_128 apds_als_clear_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xe960c9b7, 0xe0ed, 0x441e, 0xb22c, 0xd93252fa0fc6));

#endif

#ifdef BMP280

static struct bt_uuid_128 bmp280_primary_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xf4356abe, 0xb85f, 0x47c7, 0xab4e, 0x54df8f4ad025));
#endif

#ifdef LSM6DS33

//@brief  UUID for clear_als apds sensor data: e82bd800-c62c-43d5-b03f-c7381b38892a
static struct bt_uuid_128 lsm6ds33_primary_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xe82bd800, 0xc62c, 0x43d5, 0xb03f, 0xc7381b38892a));

//@brief  UUID for clear_als apds sensor data: 461d287d-1ccd-46bf-8498-60139deeeb27
static struct bt_uuid_128 lsm6ds33_accl_x_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0x461d287d, 0x1ccd, 0x46bf, 0x8498, 0x60139deeeb27));


//@brief  UUID for clear_als apds sensor data: a32f4917-d566-4273-b435-879eb85bd5cd
static struct bt_uuid_128 lsm6ds33_accl_y_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xa32f4917, 0xd566, 0x4273, 0xb435, 0x879eb85bd5cd));

//@brief  UUID for clear_als apds sensor data: e6837dcc-ff0b-4329-a271-c3269c61b10d
static struct bt_uuid_128 lsm6ds33_accl_z_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xe6837dcc, 0xff0b, 0x4329, 0xa271, 0xc3269c61b10d));

//@brief  UUID for clear_als apds sensor data: 54adba22-25c7-49d2-b4be-dbbb1a77efa3
static struct bt_uuid_128 lsm6ds33_gyro_x_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x54adba22, 0x25c7, 0x49d2, 0xb4be, 0xdbbb1a77efa3));

//@brief  UUID for clear_als apds sensor data: 67b2890f-e716-45e8-a8fe-4213db675224
static struct bt_uuid_128 lsm6ds33_gyro_y_uuid = BT_UUID_INIT_128(
BT_UUID_128_ENCODE(0x67b2890f, 0xe716, 0x45e8, 0xa8fe, 0x4213db675224));

//@brief  UUID for clear_als apds sensor data: af11d0a8-169d-408b-9933-fefd482cdcc6
static struct bt_uuid_128 lsm6ds33_gyro_z_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xaf11d0a8, 0x169d, 0x408b, 0x9933, 0xfefd482cdcc6));

#endif


#ifdef SCD41
 
//@brief  UUID for clear_als apds sensor data: fb3047b4-df00-4eb3-9587-3b00e5bb5791
static struct bt_uuid_128 scd41_primary_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xfb3047b4, 0xdf00, 0x4eb3, 0x9587, 0x3b00e5bb5791));

//@brief  UUID for clear_als apds sensor data: b82febf7-93f8-489e-8f52-b4797e33aab1
static struct bt_uuid_128 scd41_CO2_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0xb82febf7, 0x93f8, 0x93f8, 0x8f52, 0xb4797e33aab1));
#endif

#ifdef DS18B20
//@brief  UUID for clear_als apds sensor data: 8121b46f-56ce-487f-9084-5330700681d5
static struct bt_uuid_128 ds18b_primary_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x8121b46f, 0x56ce, 0x487f, 0x9084, 0x5330700681d5));

//@brief  UUID for clear_als apds sensor data: 9dbd0e19-372d-4c7e-a1cb-9a6a8e75bf0f
static struct bt_uuid_128 ds18b_temp_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x9dbd0e19, 0x372d, 0x4c7e, 0xa1cb, 0x9a6a8e75bf0f));
#endif

static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);

	bool notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    notif_enabled ? "enabled" : "disabled";

	// LOG_INF("HRS notifications %s", notif_enabled ? "enabled" : "disabled");
}

#ifdef APDS9960
/**
 * @brief  Struct bt_gatt_cpf to construct the charactersitics presentation format.
 * @note   PROXIMITY data is only 8 bits long and the unit is not specified in the data sheet.
 */
static const struct bt_gatt_cpf proximity = {
	.format = CPF_FORMAT_UINT8,
	.unit = CPF_UNIT_NO_UNIT,
};

/**
 * @brief  Struct bt_gatt_cpf to construct the charactersitics presentation format.
 * @note   ALS data is 16 bits long and the unit is not specified in the data sheet.
 */
static const struct bt_gatt_cpf als = {
	.format = CPF_FORMAT_UINT16,
	.unit = CPF_UNIT_NO_UNIT,
};

#endif

#ifdef LSM6DS33
/**
 * @brief  Struct bt_gatt_cpf to construct the charactersitics presentation format.
 * @note   Accelerometer data is 16 bits long and the unit is m/s^2.
 */
static const struct bt_gatt_cpf accel = {
	.format = CPF_FORMAT_UINT16,
	.unit = CPF_UNIT_ACCEL,
};

static const struct bt_gatt_cpf gyro = {
	.format = CPF_FORMAT_UINT16,
	.unit = CPF_UNIT_ANG_VEL,
};
#endif

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

#ifdef APDS9960
// Primary Service for APDS9960 sensor.
	BT_GATT_PRIMARY_SERVICE(&apds_uuid),
// Characteristic proximity value and descriptors for unit and format. attr[8]
	BT_GATT_CHARACTERISTIC(&apds_prox_uuid.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),	
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&proximity),

// Characteristic red_als data value and descriptors for unit and format. att[12]
	BT_GATT_CHARACTERISTIC(&apds_als_red_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&als),

// Characteristic green_als data value and descriptors for unit and format. attr[16]
	BT_GATT_CHARACTERISTIC(&apds_als_green_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&als),

// Characteristic blue_als data value and descriptors for unit and format. attr[20]
	BT_GATT_CHARACTERISTIC(&apds_als_blue_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&als),

// Characteristic blue_als data value and descriptors for unit and format. attr[24]
	BT_GATT_CHARACTERISTIC(&apds_als_clear_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&als),
#endif

#ifdef BMP280
	BT_GATT_PRIMARY_SERVICE(&bmp280_primary_uuid),
// Caharactersitic temp. value. attrs[29]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

// charactersitic humidity value. attrs[32]
	BT_GATT_CHARACTERISTIC(BT_UUID_PRESSURE, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif

#ifdef LSM6DS33
	BT_GATT_PRIMARY_SERVICE(&lsm6ds33_primary_uuid),
// Characteristic green_als data value and descriptors for unit and format. attr[36]
	BT_GATT_CHARACTERISTIC(&lsm6ds33_accl_x_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&accel),

// Characteristic blue_als data value and descriptors for unit and format. attr[40]
	BT_GATT_CHARACTERISTIC(&lsm6ds33_accl_y_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&accel),

// Characteristic blue_als data value and descriptors for unit and format. attr[44]
	BT_GATT_CHARACTERISTIC(&lsm6ds33_accl_z_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&accel),

// Characteristic green_als data value and descriptors for unit and format. attr[48]
	BT_GATT_CHARACTERISTIC(&lsm6ds33_gyro_x_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&gyro),

// Characteristic blue_als data value and descriptors for unit and format. attr[52]
	BT_GATT_CHARACTERISTIC(&lsm6ds33_gyro_y_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&gyro),

// Characteristic blue_als data value and descriptors for unit and format. attr[56]
	BT_GATT_CHARACTERISTIC(&lsm6ds33_gyro_z_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&gyro),

#endif

#ifdef SCD41
	BT_GATT_PRIMARY_SERVICE(&scd41_primary_uuid),
// Caharactersitic Co2 value. attrs[61]
	BT_GATT_CHARACTERISTIC(&scd41_CO2_uuid.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&als),

// charactersitic humidity value. attrs[65]
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),

	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

// charactersitic temperature value. attrs[68]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),

	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif

#ifdef DS18B20
	BT_GATT_PRIMARY_SERVICE(&ds18b_primary_uuid),
// Caharactersitic Co2 value. attrs[72]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	// BT_GATT_CPF(&als),

#endif
);


#ifdef SHT31
/**
 * @brief  To notify the temperature and humidity value from the sht sensor to ble.
 * @note   Same can be implemented for different sensors. Not sure why humidity value has attrs[4].
 * @retval None
 */
void ess_notify(void)
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

#ifdef APDS9960

/**
 * @brief  Notify the client the change in the proximity and ALS sensor value.
 */
void apds9960_notify(void)
{
	static apds9960_t sensor_value;
	uint8_t prox;
	int16_t red, green, blue, clear;
	read_proximity_data(&sensor_value);
	read_als_data(&sensor_value);
	prox = sensor_value.proximity;
	red = sensor_value.red;
	green = sensor_value.green;
	blue = sensor_value.blue;
	clear = sensor_value.clear;

	bt_gatt_notify(NULL, &ess_svc.attrs[8], &sensor_value.proximity, sizeof(sensor_value.proximity));
	bt_gatt_notify(NULL, &ess_svc.attrs[12], &sensor_value.red, sizeof(sensor_value.proximity));
	bt_gatt_notify(NULL, &ess_svc.attrs[16], &sensor_value.green, sizeof(sensor_value.proximity));
	bt_gatt_notify(NULL, &ess_svc.attrs[20], &sensor_value.blue, sizeof(sensor_value.proximity));
	bt_gatt_notify(NULL, &ess_svc.attrs[24], &sensor_value.clear, sizeof(sensor_value.proximity));
	return;
}
#endif

#ifdef BMP280

void bmp280_notify(void){
	static bmp280_t sensor_value;
	uint16_t temperature;
	uint32_t pressure;
	bmp_read_press_temp_data(&sensor_value);
	temperature = (uint16_t)((sensor_value.temperature)*100);
	pressure= (uint32_t)((sensor_value.pressure)*10);
	bt_gatt_notify(NULL, &ess_svc.attrs[29], &temperature, sizeof(temperature));
	bt_gatt_notify(NULL, &ess_svc.attrs[32], &pressure, sizeof(pressure));
	return;
}
#endif

#ifdef LSM6DS33
void lsm6ds33_notify(void){
	static lsm6ds33_t sensor_value;
	int16_t gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
	read_burst_data(&sensor_value);
	gyroX = (int16_t) (((sensor_value.gyroX)*100)+32768);
	gyroY = (int16_t) ((sensor_value.gyroY)*100+32768);
	gyroZ = (int16_t) ((sensor_value.gyroZ)*100+32768);

	accelX = (int16_t)((sensor_value.accelX)*100+32768);
	accelY = (int16_t)((sensor_value.accelY)*100+32768);
	accelZ = (int16_t)((sensor_value.accelZ)*100+32768);

	// gyroX = ~(gyroX-1);
	// gyroY = ~(gyroY-1);
	// gyroZ = ~(gyroZ-1);

	// accelX = ~(accelX-1);
	// accelY = ~(accelY-1);
	// accelZ = ~(accelZ-1);

	bt_gatt_notify(NULL, &ess_svc.attrs[36], &accelX, sizeof(accelX));
	bt_gatt_notify(NULL, &ess_svc.attrs[40], &accelY, sizeof(accelY));
	bt_gatt_notify(NULL, &ess_svc.attrs[44], &accelZ, sizeof(accelZ));

	bt_gatt_notify(NULL, &ess_svc.attrs[48], &gyroX, sizeof(gyroX));
	bt_gatt_notify(NULL, &ess_svc.attrs[52], &gyroY, sizeof(gyroY));
	bt_gatt_notify(NULL, &ess_svc.attrs[56], &gyroZ, sizeof(gyroZ));
	return;
}
#endif


#ifdef SCD41

void scd41_notify(void){
	static scd41_t sensor_value;
	measure_single_shot(&sensor_value);
	uint16_t Co2 = sensor_value.Co2;
	uint16_t temp, hum;
	temp = (uint16_t)(sensor_value.temp*100);
	hum = (uint16_t)(sensor_value.hum*100);
	bt_gatt_notify(NULL, &ess_svc.attrs[61], &Co2, sizeof(Co2));
	bt_gatt_notify(NULL, &ess_svc.attrs[65], &hum, sizeof(hum));
	bt_gatt_notify(NULL, &ess_svc.attrs[68], &temp, sizeof(temp));
}

#endif

#ifdef DS18B20

void ds18b_notify(void){
	static float sensor_value;
	requestTemperatures();
	sensor_value = getTempCByIndex(0);
	static uint16_t some;
	some = (uint16_t)((sensor_value)*100);
	bt_gatt_notify(NULL, &ess_svc.attrs[72], &some, sizeof(some));
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
	printk("Badhu chalu to thai gayu\n");
#ifdef APDS9960
	enable_apds_sensor();
#endif

#ifdef BMP280
	read_calibration_registers();
#endif

#ifdef LSM6DS33
	check_device();
	lsm6ds33_init();
#endif

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
        delay(1000);

#ifdef SHT31
        ess_notify();
#endif

#ifdef APDS9960
		apds9960_notify();
#endif

#ifdef BMP280
		bmp280_notify();
#endif

#ifdef LSM6DS33
		lsm6ds33_notify();
#endif
#ifdef SCD41
		scd41_notify();
#endif
#ifdef DS18B20
		ds18b_notify();
#endif
    }
}