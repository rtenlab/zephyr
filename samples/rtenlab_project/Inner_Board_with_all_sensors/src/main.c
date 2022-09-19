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
#include <math.h>

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
#include "blinky/blink.c"
#include "battery/battery.h"


#define SHT31
#define APDS9960
#define BMP280
#define LSM6DS33
#define SCD41
#define DS18B20
#define BME680
#define BLE
#define BATTERY
// #define MAIN_DEBUG

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
#define CPF_VOLTAGE_UNIT	0x2B18

volatile bool BLE_isConnected = false;
volatile bool notif_enabled;
volatile bool bme680_is_first_reading=true;

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
#endif

#ifdef BME680
//@brief  UUID for bme680 apds sensor data: 54adba22-25c7-49d2-b4be-dbbb1a77efa3
static struct bt_uuid_128 bme680_primary_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x54adba22, 0x25c7, 0x49d2, 0xb4be, 0xdbbb1a77efa3));

//@brief  UUID for bme680 apds sensor data: 67b2890f-e716-45e8-a8fe-4213db675224
static struct bt_uuid_128 bme680_gas_uuid = BT_UUID_INIT_128(
BT_UUID_128_ENCODE(0x67b2890f, 0xe716, 0x45e8, 0xa8fe, 0x4213db675224));
#endif

#ifdef BATTERY
//@brief  UUID for battery level data: c9e3205e-f994-4ff0-8300-9b703aecae08
static struct bt_uuid_128 battery_primary_uuid = BT_UUID_INIT_128(
BT_UUID_128_ENCODE(0xc9e3205e, 0xf994, 0x4ff0, 0x8300, 0x9b703aecae08));

// @brief UUID for battery secondary level data: 3d84bece-189c-4bc7-9f10-512173ed8eaa
static struct bt_uuid_128 battery_secondary_uuid = BT_UUID_INIT_128(
	BT_UUID_128_ENCODE(0x3d84bece,0x189c,0x4bc7,0x9f10,0x512173ed8eaa));
#endif

static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	// printk("hrmc_ccc_cfg_changed: This function was called and hence notifications should be ON!!!\n");
	notif_enabled = (value == BT_GATT_CCC_NOTIFY);
    // notif_enabled ? "enabled" : "disabled";

	// LOG_INF("HRS notifications %s", notif_enabled ? "enabled" : "disabled");
}

#ifdef APDS9960

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

#ifdef BATTERY
/**
 * @brief  Struct bt_gatt_cpf to construct the charactristics presentation format.
 * @note   The sensor uses uint16_t for data and the unit is simple voltage
 * @retval None
 */
static const struct bt_gatt_cpf battery = {
	.format = CPF_FORMAT_UINT16,
	.unit = CPF_VOLTAGE_UNIT,
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

#ifdef BATTERY
	// Primary service for battery level.
	BT_GATT_PRIMARY_SERVICE(&battery_primary_uuid),
	// Charateristic battery level value and cccd. attr[53]
	BT_GATT_CHARACTERISTIC(&battery_secondary_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&battery),
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
	// Get the temperature and humidity value from the sensor.
	read_temp_hum(&sensor_value);
	// Convert the float values to 2 pt. precision uint16_t.
    temp_value = (uint16_t)((sensor_value.temp)*100);
	hum_value = (uint16_t)((sensor_value.humidity)*100);
    bt_gatt_notify(NULL, &ess_svc.attrs[1], &temp_value, sizeof(temp_value));
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[4], &hum_value, sizeof(hum_value));
	k_sleep(K_MSEC(500));
	printk("SHT Notified!!!\n");
}
#endif

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

#ifdef BMP280

void bmp280_notify(void){
	static bmp280_t sensor_value;
	uint16_t temperature;
	uint32_t pressure;
	// Get the temperature and pressure value from the sensor.
	bmp_read_press_temp_data(&sensor_value);
	// Convert the float values to 2 pt. precision uint16_t.
	temperature = (uint16_t)((sensor_value.temperature)*100);
	pressure= (uint32_t)((sensor_value.pressure)*10);
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[13], &temperature, sizeof(temperature));
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[16], &pressure, sizeof(pressure));
	k_sleep(K_MSEC(500));
	printk("BMP Notified!!!\n");
	return;
}
#endif

#ifdef LSM6DS33
void lsm6ds33_notify(void){
	// static lsm6ds33_t sensor_value;
	float accelX, accelY, accelZ;
	int16_t totalX, totalY, totalZ, count=0;
	int16_t finalX=0, finalY=0, finalZ=0;
	
		// Initialize the sensor by setting necessary parameters for gyroscope and accelerometer
	lsm6ds33_init();

	accel_set_power_mode(ACCEL_LOW_POWER_MODE);
	gyro_set_power_mode(GYRO_LOW_POWER_MODE);

	printk("Waiting for watermark...\nIf you want to see some changes in the readings change the orientation of the board right now!!!\n");

		//  Check if the FIFO_FULL Flag is set or not.
	while((lsm6ds33_fifo_status() & FIFO_FULL) == 0) { };
	count =0; totalX=0; totalY=0; totalZ=0;
	while( (lsm6ds33_fifo_status()& FIFO_EMPTY) == 0){
		accelX = lsm6ds33_fifo_get_accel_data(lsm6ds33_fifo_read());
		totalX +=accelX;
		// printk("AccelX_Raw: %f\n", accelX);
		accelY = lsm6ds33_fifo_get_accel_data(lsm6ds33_fifo_read());
		totalY +=accelY;
		// printk("AccelY_Raw: %f\n", accelY);
		accelZ = lsm6ds33_fifo_get_accel_data(lsm6ds33_fifo_read());
		totalZ +=accelZ;
		// printk("AccelZ_Raw: %f\n", accelZ);
		count++;
		// printk("AccelX_Raw: %f\n", accelX);
		// printk("AccelY_Raw: %f\n", lsm6ds33_fifo_get_accel_data(lsm6ds33_fifo_read()));
		// printk("AccelZ_Raw: %f\n", lsm6ds33_fifo_get_accel_data(lsm6ds33_fifo_read()));
		delay(50);
	}
	lsm6ds33_fifo_change_mode(0);

	// 327968 is added to convert the negative value to positive. Opposite of this is done on raspberry side.
	finalX = (int16_t)((totalX/count)*100+32768);
	finalY = (int16_t)((totalY/count)*100+32768);
	finalZ = (int16_t)((totalZ/count)*100+32768);

	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[20], &finalX, sizeof(finalX));
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[24], &finalY, sizeof(finalY));
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[28], &finalZ, sizeof(finalZ));
	printk("LSM Notified");
	k_sleep(K_MSEC(500));
	return;
}
#endif


#ifdef SCD41

void scd41_notify(void){
	static scd41_t sensor_value;
	// Get the values from the sensor.
	measure_single_shot(&sensor_value);
	uint16_t Co2 = sensor_value.Co2;
	uint16_t temp, hum;
	temp = (uint16_t)(sensor_value.temp*100);
	hum = (uint16_t)(sensor_value.hum*100);
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[33], &Co2, sizeof(Co2));
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[37], &hum, sizeof(hum));
	k_sleep(K_MSEC(500));
	bt_gatt_notify(NULL, &ess_svc.attrs[40], &temp, sizeof(temp));
	k_sleep(K_MSEC(500));
	printk("SCD Notified!!!\n");
}

#endif

#ifdef DS18B20

void ds18b_notify(void){
	// Initialize sensor values.
	static float sensor_value;
	static uint16_t final_value;
	// Notify Sensor to start collecting temperature data.
	requestTemperatures();
	// We just have one sensor so get the value of the first sensor.
	sensor_value = getTempCByIndex(0);
	// Can't send a float value so convert the actual temperature to whole number with 2 point precision.
	final_value= (uint16_t)((sensor_value)*100);
	// Notify the respective Handler about the change in values.
	bt_gatt_notify(NULL, &ess_svc.attrs[44], &final_value, sizeof(final_value));
	k_sleep(K_MSEC(500));
	return;
}
#endif

#ifdef BME680
void bme680_notify(){
	// we only need to do this first time. NO need to do that again.
	static bme680_gas_par_t gascalib;
	static bme680_gas_data_t gasdata;
	if(bme680_is_first_reading==true){	
		// Get the Gas Calibration data
		if(!bme680_get_gas_calib_data(&gascalib)){
			printk("BME680: Gas Calib data returned with false at line %d\n", __LINE__);
		}
		// Set the heater Configuration
		if(bme680_set_heater_conf(350,750,&gascalib)!=true){
			printk("Set Heater Conf returned with False\n");
		}
		bme680_is_first_reading=false;
		#ifdef DEBUG
		printk("%d\n",gascalib.para1);
		printk("%d\n",gascalib.para2);
		printk("%d\n",gascalib.para3);
		printk("%d\n",gascalib.res_heat_range);
		printk("%d\n",gascalib.res_heat_val);
		printk("%d\n",gascalib.range_sw_err);
		#endif
	}
	// This marks the start of our data reading stage.
	// Change the power mode to forced mode.
	printk("Starting the BME680 measurement\n");
	bme680_set_power_mode(BME680_FORCED_MODE);
	delay(900);
	#ifdef DEBUG
	printk("Before: ");
	// Check if we are getting a new data or no.
	bme680_check_new_data();
	#endif
	// Wait for the heater to heat.
	bme680_get_raw_gas_data(&gasdata);
	#ifdef DEBUG
	printk("After: ");
	// Check again if we are getting a new data or no.
	bme680_check_new_data();
	#endif
	// Get the raw data from the registers.
	static uint32_t value =0;
	value = (bme680_calc_gas_resistance(&gasdata, &gascalib)/1000);
	printk("%d\n", value);
	bt_gatt_notify(NULL, &ess_svc.attrs[48], &value, sizeof(value));
	return;

}
#endif

#ifdef BATTERY
void batt_notify(){
	// Get the battery voltage in volts
	float batt = battery_sample();

	if(batt < 0){
		led_on_blink0(false);
		printk("failed to read battery voltage: %d", (int)batt);
	}
	// Convert the float to 2 pt precision integer value.
	uint16_t batt_int = (uint16_t)((batt)*100);
	bt_gatt_notify(NULL, &ess_svc.attrs[53],&batt_int, sizeof(batt_int));
	return;
}
#endif

static const struct bt_data ad[] = {
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA_BYTES(BT_DATA_UUID16_ALL,
		      BT_UUID_16_ENCODE(BT_UUID_ESS_VAL)),
};

static void connected(struct bt_conn *conn, uint8_t err)
{
	BLE_isConnected=true;
	printk("_isConnected set to: %d\n", BLE_isConnected);
	if (err) {
		printk("Connection failed (err 0x%02x)\n", err);
	} else {
		printk("Connected\n");
	}
	// Turn off the LED to notify we are connected!
	led_on_blink1(false);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	BLE_isConnected=false;
	notif_enabled=false;
	printk("_isConnected set to: %d\n", BLE_isConnected);
	// Not sure if this is needed!
	int8_t err = bt_conn_disconnect(conn, 0x08);
	if(err){
		printk("Disconnected call returned with %d\n", err);
	}
	err = bt_le_adv_start(BT_LE_ADV_CONN_NAME, ad, ARRAY_SIZE(ad), NULL, 0);
	if(err){
		printk("Advertising failed to start (err %d)\n", err);
	}
	printk("Disconnected (reason 0x%02x)\n", reason);
	// Turn ON the LED to notify we are not connected!
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
	// Start the LED immediately to signal it is not connected right now.
	led_on_blink1(true);
}




void main(void)
{

	// enable_uart_console();
	configure_device();
	
	// For battery calculation!
	int rc = battery_measure_enable(true);
	if (rc != 0) {
		printk("Failed initialize battery measurement: %d\n", rc);
		return;
	}

#ifdef BLE
    int err;
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}

	bt_ready();

	bt_conn_cb_register(&conn_callbacks);
	printk("Call backs registered\n");
	// LOG_INF("Call backs registered!!!\n");
#endif	

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
	dev_ds18b20 = device_get_binding(DS_SENSOR);
	if (dev_ds18b20 == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev_ds18b20, DS_SENSOR_PIN, GPIO_OUTPUT);
	if (ret < 0) {
		return;
	}
	DallasTemperature_begin();
#endif

    while (1) {
		if(BLE_isConnected){// Wait untill we have BLE_isCONNECTED as true.
		while(!notif_enabled){;}
		k_sleep(K_SECONDS(5));
		led_on_blink0(true);
		printk("Sending data currently at the start of the loop!!!\n");
	
	#ifdef BATTERY
		batt_notify();
	#endif
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
		led_on_blink1(false);
		led_on_blink1(true);
		lsm6ds33_notify();
		led_on_blink1(false);
	#endif

	#ifdef SCD41
		scd41_notify();
	#endif
	#ifdef DS18B20
		ds18b_notify();
	#endif
	#ifdef BME680
		bme680_notify();
	#endif
		led_on_blink0(false);
		#ifdef MAIN_DEBUG
		k_sleep(K_SECONDS(70));
		#else
		if (BLE_isConnected) {
			k_sleep(K_MINUTES(20));
		}		
		#endif
		}//End of if
	}// End of while
}// End of main