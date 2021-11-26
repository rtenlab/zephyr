#right your most recent used here
nothing:
	./make.sh all_sensors
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





