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
#include "bme680/bme680.h"

#define SHT31
#define APDS9960
#define BMP280
#define LSM6DS33
// #define SCD41
// #define DS18B20
// #define BME680
// Defines to get the format of the data sent using custom characteristics UUID
#define CPF_FORMAT_UINT8 	0x04
#define CPF_FORMAT_UINT16 	0x06
#define CPF_FORMAT_UINT32	0x08

// Defines to get the unitsof the data sent using custom characteristics UUID

#define CPF_UNIT_NO_UNIT 	0x2700
#define CPF_UNIT_METER 		0x2701
#define CPF_UNIT_ACCEL 		0x2713
#define CPF_UNIT_ANG_VEL	0x2743
#define CPF_UNIT_ELEC_RES		0x272A

#ifdef SHT31

// @brief 57812a99-9146-4e72-a4b7-5159632dee90
static struct bt_uuid_128 sht_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0x57812a99, 0x9146, 0x4e72, 0Xa4b7,  0X5159632dee90));

#endif

#ifdef APDS9960

// @brief  UUID for apds sensor data: ebcc60b7-974c-43e1-a973-426e79f9bc6c 
static struct bt_uuid_128 apds_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xebcc60b7, 0x974c, 0x43e1, 0Xa973,  0X426e79f9bc6c));

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

#ifdef BME680
//@brief  UUID for bme680 apds sensor data: 54adba22-25c7-49d2-b4be-dbbb1a77efa3
static struct bt_uuid_128 bme680_primary_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x54adba22, 0x25c7, 0x49d2, 0xb4be, 0xdbbb1a77efa3));

//@brief  UUID for bme680 apds sensor data: 67b2890f-e716-45e8-a8fe-4213db675224
static struct bt_uuid_128 bme680_gas_uuid = BT_UUID_INIT_128(
BT_UUID_128_ENCODE(0x67b2890f, 0xe716, 0x45e8, 0xa8fe, 0x4213db675224));

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

#ifdef BME680

/**
 * @brief  Struct bt_gatt_cpf to construct the charactersitics presentation format.
 * @note   This sensor uses 32bit data and the unit is electrical resistance
 */
static const struct bt_gatt_cpf bme680_gas = {
	.format = CPF_FORMAT_UINT32,
	.unit = CPF_UNIT_ELEC_RES,
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

// Characteristic blue_als data value and descriptors for unit and format. attr[8]
	BT_GATT_CHARACTERISTIC(&apds_als_clear_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&als),
#endif

#ifdef BMP280
	BT_GATT_PRIMARY_SERVICE(&bmp280_primary_uuid),
// Caharactersitic temp. value. attrs[13]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

// charactersitic humidity value. attrs[16]
	BT_GATT_CHARACTERISTIC(BT_UUID_PRESSURE, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif

#ifdef LSM6DS33
	BT_GATT_PRIMARY_SERVICE(&lsm6ds33_primary_uuid),
// Characteristic green_als data value and descriptors for unit and format. attr[20]
	BT_GATT_CHARACTERISTIC(&lsm6ds33_accl_x_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&accel),

// Characteristic blue_als data value and descriptors for unit and format. attr[24]
	BT_GATT_CHARACTERISTIC(&lsm6ds33_accl_y_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&accel),

// Characteristic blue_als data value and descriptors for unit and format. attr[28]
	BT_GATT_CHARACTERISTIC(&lsm6ds33_accl_z_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&accel),

#endif
#ifdef BME680
// Primary Service for BME680 sensor.
	BT_GATT_PRIMARY_SERVICE(&bme680_primary_uuid),

// Characteristic blue_als data value and descriptors for unit and format. attr[48]
	BT_GATT_CHARACTERISTIC(&bme680_gas_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&bme680_gas),

#endif

#ifdef SCD41
	BT_GATT_PRIMARY_SERVICE(&scd41_primary_uuid),
// Caharactersitic Co2 value. attrs[33]
	BT_GATT_CHARACTERISTIC(&scd41_CO2_uuid.uuid, BT_GATT_CHRC_NOTIFY,
			       BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&als),

// charactersitic humidity value. attrs[37]
	BT_GATT_CHARACTERISTIC(BT_UUID_HUMIDITY, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),

	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),

// charactersitic temperature value. attrs[40]
	BT_GATT_CHARACTERISTIC(BT_UUID_TEMPERATURE, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),

	BT_GATT_CCC(hrmc_ccc_cfg_changed,
		    BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
#endif

#ifdef DS18B20
	BT_GATT_PRIMARY_SERVICE(&ds18b_primary_uuid),
// Caharactersitic Co2 value. attrs[44]
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

#ifdef APDS9960

/**
 * @brief  Notify the client the change in the proximity and ALS sensor value.
 */
void apds9960_notify(void)
{
	static apds9960_t sensor_value;
	uint8_t prox;
	int16_t red, green, blue, clear;
	read_als_data(&sensor_value);
	clear = sensor_value.clear;

	bt_gatt_notify(NULL, &ess_svc.attrs[8], &sensor_value.clear, sizeof(sensor_value.clear));
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
	bt_gatt_notify(NULL, &ess_svc.attrs[13], &temperature, sizeof(temperature));
	bt_gatt_notify(NULL, &ess_svc.attrs[16], &pressure, sizeof(pressure));
	return;
}
#endif

#ifdef LSM6DS33
void lsm6ds33_notify(void){
	static lsm6ds33_t sensor_value;
	int16_t gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
	read_burst_data(&sensor_value);

	accelX = (int16_t)((sensor_value.accelX)*100+32768);
	accelY = (int16_t)((sensor_value.accelY)*100+32768);
	accelZ = (int16_t)((sensor_value.accelZ)*100+32768);


	bt_gatt_notify(NULL, &ess_svc.attrs[20], &accelX, sizeof(accelX));
	bt_gatt_notify(NULL, &ess_svc.attrs[24], &accelY, sizeof(accelY));
	bt_gatt_notify(NULL, &ess_svc.attrs[28], &accelZ, sizeof(accelZ));

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
	bt_gatt_notify(NULL, &ess_svc.attrs[33], &Co2, sizeof(Co2));
	bt_gatt_notify(NULL, &ess_svc.attrs[37], &hum, sizeof(hum));
	bt_gatt_notify(NULL, &ess_svc.attrs[40], &temp, sizeof(temp));
}

#endif

#ifdef DS18B20

void ds18b_notify(void){
	static float sensor_value;
	requestTemperatures();
	sensor_value = getTempCByIndex(0);
	static uint16_t some;
	some = (uint16_t)((sensor_value)*100);
	bt_gatt_notify(NULL, &ess_svc.attrs[44], &some, sizeof(some));
	return;

}
#endif

#ifdef BME680

void bme680_notify(bool send){
	bme680_gas_par_t gascalib;
	bme680_gas_data_t gasdata;
	if(!bme680_get_gas_calib_data(&gascalib)){
		printk("BME680: Gas Calib data returned with false at line %d\n", __LINE__);
	}
#ifdef DEBUG
	printk("%d\n",gascalib.para1);
	printk("%d\n",gascalib.para2);
	printk("%d\n",gascalib.para3);
	printk("%d\n",gascalib.res_heat_range);
	printk("%d\n",gascalib.res_heat_val);
	printk("%d\n",gascalib.range_sw_err);
#endif
	// Set the coil to start heating.
	bme680_set_heater_conf(320,150,&gascalib);
	// Change the power mode to forced mode.
	bme680_set_power_mode(1);
	#ifdef DEBUG
	printk("Before: ");
	// Check if we are getting a new data or no.
	bme680_check_new_data();
	#endif
	// Wait for the heater to heat.
	delay(200);
	#ifdef DEBUG
	printk("After: ");
	// Check again if we are getting a new data or no.
	bme680_check_new_data();
	#endif
	// Get the raw data from the registers.
	bme680_get_raw_gas_data(&gasdata);
	static uint32_t value =0;
	value = (bme680_calc_gas_resistance(&gasdata, &gascalib));
	if(send)
		bt_gatt_notify(NULL, &ess_svc.attrs[33], &value, sizeof(value));
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
        sht_notify();
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
#ifdef BME680
		bme680_notify(true);
#endif
    }
}