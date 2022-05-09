outer_board:
	./make.sh Outer_Board_with_DS_sensors
	./flash.sh

inner_board:
	./make.sh Inner_Board_with_all_sensors
	./flash.sh

battery:
	./make.sh battery
	./flash.sh
bme:
	./make.sh bme680
	./flash.sh
i2c:
	./make.sh i2c_scanner
	./flash.sh
ds:
	./make.sh ds18b20
	./flash.sh
sync_dev:
	./make.sh synchronization_dev_implementation
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





