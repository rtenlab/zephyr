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

#right your most recent used here
all:
	./make.sh lsm6ds33
	./flash.sh


