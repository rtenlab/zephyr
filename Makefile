ds_ble:
	./make.sh DS18B20_Ble
	./flash.sh
final:
	./make.sh final_adafruit_peripheral_bluetooth_multi
	./flash.sh
thread_analyzer:
	./make.sh thread_analysis_testbed
	./flash.sh
sync:
	./make.sh synchronization
	./flash.sh
bme:
	./make.sh bme680
	./flash.sh
i2c:
	./make.sh i2c_scanner
	./flash.sh
per:
	./make.sh ble_conn
	./flash.sh
button:
	./make.sh button
	./flash.sh
ds:
	./make.sh ds18b20
	./flash.sh
bt:
	./make.sh beacon_dev_implementation
	./flash.sh
sync_dev:
	./make.sh synchronization_dev_implementation
	./flash.sh
all:
	./make.sh all_sensors
	./flash.sh
lsm:
	./make.sh lsm6ds33
	./flash.sh

sht:
	./make.sh sht31
	./flash.sh

apds:
	./make.sh apds9960
	./flash.sh

bmp:
	./make.sh bmp280
	./flash.sh

scd:
	./make.sh scd41
	./flash.sh





