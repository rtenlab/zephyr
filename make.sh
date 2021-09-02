# Script file to make the zephyr image. 

# Usage: Give this file executable right by running "chmod +x make.sh"
#        To compile Blinky stored in "zephyr/samples/basic/blinky" run "./make.sh basic blinky"

rm -rf build
west build -b adafruit_feather_nrf52840_sense samples/"$1"/"$2"
