lsm:
	./make.sh lsm6ds33
	sudo ./flash.sh

sht:
	./make.sh sht31
	sudo ./flash.sh

apds:
	./make.sh apds9960
	sudo ./flash.sh

bmp:
	./make.sh bmp280
	sudo ./flash.sh

#right your most recent used here
all:
	./make.sh lsm6ds33
	sudo ./flash.sh


