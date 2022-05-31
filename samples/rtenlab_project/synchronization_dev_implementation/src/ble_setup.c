#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/conn.h>
#include <bluetooth/uuid.h>
#include <bluetooth/gatt.h>
#include <usb/usb_device.h>
#include "blink.c"
#define BLE_SHT31
#define BLE_APDS9960
#define BLE_BMP280
#define BLE_LSM6DS33

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

#define SHT_TEMP_BLE_HANDLE 1		// 1
#define SHT_HUM_BLE_HANDLE (SHT_TEMP_BLE_HANDLE+3)  // 4
#define APDS_BLE_HANDLE (SHT_HUM_BLE_HANDLE+4)		// 8
#define BMP_TEMP_BLE_HANDLE (APDS_BLE_HANDLE+5)     // 13
#define BMP_PRESS_BLE_HANDLE (BMP_TEMP_BLE_HANDLE+3)  // 16
#define LSM_ACCELX_BLE_HANDLE (BMP_PRESS_BLE_HANDLE+4) //20
#define LSM_ACCELY_BLE_HANDLE (LSM_ACCELX_BLE_HANDLE+4) //24
#define LSM_ACCELZ_BLE_HANDLE (LSM_ACCELY_BLE_HANDLE+4) //28


volatile bool BLE_isConnected = false;
volatile bool notif_enabled = false;


#ifdef BLE_SHT31

// @brief 57812a99-9146-4e72-a4b7-5159632dee90
static struct bt_uuid_128 sht_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0x57812a99, 0x9146, 0x4e72, 0Xa4b7,  0X5159632dee90));

#endif

#ifdef BLE_APDS9960

// @brief  UUID for apds sensor data: ebcc60b7-974c-43e1-a973-426e79f9bc6c 
static struct bt_uuid_128 apds_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xebcc60b7, 0x974c, 0x43e1, 0Xa973,  0X426e79f9bc6c));

// 	@brief  UUID for clear_als apds sensor data: e960c9b7-e0ed-441e-b22c-d93252fa0fc6

static struct bt_uuid_128 apds_als_clear_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xe960c9b7, 0xe0ed, 0x441e, 0xb22c, 0xd93252fa0fc6));

#endif

#ifdef BLE_BMP280

static struct bt_uuid_128 bmp280_primary_uuid = BT_UUID_INIT_128(
		BT_UUID_128_ENCODE(0xf4356abe, 0xb85f, 0x47c7, 0xab4e, 0x54df8f4ad025));
#endif

#ifdef BLE_LSM6DS33

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


static void hrmc_ccc_cfg_changed(const struct bt_gatt_attr *attr, uint16_t value)
{
	ARG_UNUSED(attr);
	notif_enabled = (value == BT_GATT_CCC_NOTIFY);
}



#ifdef BLE_APDS9960

/**
 * @brief  Struct bt_gatt_cpf to construct the charactersitics presentation format.
 * @note   ALS data is 16 bits long and the unit is not specified in the data sheet.
 */
static const struct bt_gatt_cpf als = {
	.format = CPF_FORMAT_UINT16,
	.unit = CPF_UNIT_NO_UNIT,
};

#endif

#ifdef BLE_LSM6DS33
/**
 * @brief  Struct bt_gatt_cpf to construct the charactersitics presentation format.
 * @note   Accelerometer data is 16 bits long and the unit is m/s^2.
 */
static const struct bt_gatt_cpf accel = {
	.format = CPF_FORMAT_UINT16,
	.unit = CPF_UNIT_ACCEL,
};

#endif



BT_GATT_SERVICE_DEFINE(ess_svc,
// Primary Service for SHT Sensor. Basically Tempearture and sensor value.
#ifdef BLE_SHT31
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

#ifdef BLE_APDS9960
// Primary Service for APDS9960 sensor.
	BT_GATT_PRIMARY_SERVICE(&apds_uuid),

// Characteristic blue_als data value and descriptors for unit and format. attr[8]
	BT_GATT_CHARACTERISTIC(&apds_als_clear_uuid.uuid, BT_GATT_CHRC_NOTIFY,
					BT_GATT_PERM_READ, NULL, NULL, NULL),
	BT_GATT_CCC(hrmc_ccc_cfg_changed,
			BT_GATT_PERM_READ | BT_GATT_PERM_WRITE),
	BT_GATT_CPF(&als),
#endif

#ifdef BLE_BMP280
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

#ifdef BLE_LSM6DS33
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
);


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

