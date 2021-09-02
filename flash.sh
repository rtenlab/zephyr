# This shell script will load the build directory to the Adafruit_feather_sense board.
# Give executable permissions and then just run as an executable file.

pass="your-password"        # Change the your-password field according to your system.

sudo -S < <(echo "$pass") adafruit-nrfutil dfu genpkg --dev-type 0x0052 --sd-req 0x00B6 --application build/zephyr/zephyr.hex build/zephyr/zephyr.zip 

sudo adafruit-nrfutil --verbose dfu serial -pkg build/zephyr/zephyr.zip -p /dev/ttyACM0 -b 115200 --singlebank
