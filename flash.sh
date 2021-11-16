sudo -s adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x00B6 --application build/zephyr/zephyr.hex build/zephyr/zephyr.zip

adafruit-nrfutil --verbose dfu serial -pkg build/zephyr/zephyr.zip -p /dev/ttyACM0 -b 115200 --singlebank

