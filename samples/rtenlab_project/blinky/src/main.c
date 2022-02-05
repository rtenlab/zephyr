/*
 * Copyright (c) 2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "uart_i2c.h"
#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <stdio.h>
#include <drivers/gpio.h>
#include "sht31.h"

/* 1000 msec = 1 sec */
#define SLEEP_TIME_MS   1000

/* The devicetree node identifier for the "led0" alias. */

#define LED1_NODE DT_ALIAS(led2)

#if DT_NODE_HAS_STATUS(LED1_NODE, okay)
#define LED1	DT_GPIO_LABEL(LED1_NODE, gpios)
#define LED1_PIN		DT_GPIO_PIN(LED1_NODE, gpios)
#define LED1_FLAGS	DT_GPIO_FLAGS(LED1_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led1 devicetree alias is not defined"
#define LED1	""
#define LED1_PIN	0
#define LED1_FLAGS	0
#endif
#define LOW 0
#define HIGH 1

static inline void udelay(int secs){
	k_busy_wait(secs);
}

const struct device *dev0, *dev1;

#define ONEWIRE_SEARCH 1
#define ONEWIRE_CRC 0
// global search state
unsigned char ROM_NO[8];
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
bool LastDeviceFlag;

uint8_t reset(void){
	int key = irq_lock();
	uint8_t r;
	gpio_pin_configure(dev1, LED1_PIN, GPIO_OUTPUT);
	gpio_pin_set(dev1,LED1_PIN,LOW);
	udelay(480);
	gpio_pin_set(dev1,LED1_PIN,HIGH);
	udelay(70);
	// while((gpio_pin_get(dev1,LED1_PIN))!=0){
	// 	r = gpio_pin_get(dev1,LED1_PIN);
	// 	printk("stuck in teh while loopwith value of r: %d\n",r);
	// }
	gpio_pin_configure(dev1, LED1_PIN, GPIO_INPUT);
	r = gpio_pin_get(dev1,LED1_PIN);
	udelay(410);
	irq_unlock(key);
	return !r;
}

uint8_t read_bit(void){
	uint8_t r;
	int key = irq_lock();
	gpio_pin_configure(dev1, LED1_PIN, GPIO_OUTPUT);
	gpio_pin_set(dev1,LED1_PIN,LOW);
	udelay(3);
	// gpio_pin_set(dev1,LED1_PIN,HIGH);
	gpio_pin_configure(dev1, LED1_PIN, GPIO_INPUT);
	udelay(10);	
	r = gpio_pin_get(dev1,LED1_PIN);
	irq_unlock(key);
	udelay(53);
	return r;
}

uint8_t read(void){
	uint8_t bitMask;
	uint8_t r = 0;
	for(bitMask=0x01; bitMask;bitMask<<=1){
		if(read_bit()){
			r|=bitMask;
			// udelay(1);
		}
	}
	return r;
}

void write_bit(uint8_t v){
	// printk("Outputting bit: %d\n",v);
	if(v&1){
		int key = irq_lock();
		gpio_pin_set(dev1,LED1_PIN,LOW);
		gpio_pin_configure(dev1, LED1_PIN, GPIO_OUTPUT);
		udelay(10);
		gpio_pin_set(dev1,LED1_PIN,HIGH);
		irq_unlock(key);
		udelay(55);
	}
	else{
		int key = irq_lock();
		gpio_pin_set(dev1,LED1_PIN,LOW);
		gpio_pin_configure(dev1, LED1_PIN, GPIO_OUTPUT);
		udelay(65);
		gpio_pin_set(dev1,LED1_PIN,HIGH);
		irq_unlock(key);
		udelay(5);
	}
	return;	
}

void write(uint8_t v){
	uint8_t bitMask;
	for(bitMask = 0x01; bitMask; bitMask<<=1){
		write_bit((bitMask&v)?1:0);
		// udelay(1);
	}
	return;
}

bool search(uint8_t *newAddr)
{
   bool search_mode = true;
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number;
   bool    search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = false;

   // if the last call was not the last one
   if (!LastDeviceFlag) {
      // 1-Wire reset
      if (!reset()) {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
         return false;
      }

      // issue the search command
      if (search_mode == true) {
        write(0xF0);   // NORMAL SEARCH
      } else {
        write(0xEC);   // CONDITIONAL SEARCH
      }

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = read_bit();
         cmp_id_bit = read_bit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1)) {
            break;
         } else {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit) {
               search_direction = id_bit;  // bit write value for search
            } else {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy) {
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               } else {
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);
               }
               // if 0 was picked then record its position in LastZero
               if (search_direction == 0) {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            write_bit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0) {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65)) {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0) {
            LastDeviceFlag = true;
         }
         search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0]) {
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   } else {
      for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   }
   return search_result;
}

void select(const uint8_t rom[8])
{
    uint8_t i;

    write(0x55);           // Choose ROM

    for (i = 0; i < 8; i++) write(rom[i]);
}

void reset_search()
{
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = false;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--) {
    ROM_NO[i] = 0;
    if ( i == 0) break;
  }
}
void skip()
{
    write(0xCC);           // Skip ROM
}


typedef uint8_t DeviceAddress[8];
typedef uint8_t ScratchPad[9];

// parasite power on or off
bool parasite;

// external pullup
bool useExternalPullup;
uint8_t pullupPin;

// used to determine the delay amount needed to allow for the
// temperature conversion to take place
uint8_t bitResolution;

// used to requestTemperature with or without delay
bool waitForConversion;

// used to requestTemperature to dynamically check if a conversion is complete
bool checkForConversion;

// used to determine if values will be saved from scratchpad to EEPROM on every scratchpad write
bool autoSaveScratchPad;

// count of devices on the bus
uint8_t devices;

// count of DS18xxx Family devices on bus
uint8_t ds18Count;

// Model IDs
#define DS18S20MODEL 0x10  // also DS1820
#define DS18B20MODEL 0x28  // also MAX31820
#define DS1822MODEL  0x22
#define DS1825MODEL  0x3B
#define DS28EA00MODEL 0x42

// Error Codes
#define DEVICE_DISCONNECTED_C -127
#define DEVICE_DISCONNECTED_F -196.6
#define DEVICE_DISCONNECTED_RAW -7040

// DSROM FIELDS
#define DSROM_FAMILY    0
#define DSROM_CRC       7

// Device resolution
#define TEMP_9_BIT  0x1F //  9 bit
#define TEMP_10_BIT 0x3F // 10 bit
#define TEMP_11_BIT 0x5F // 11 bit
#define TEMP_12_BIT 0x7F // 12 bit

// OneWire commands
#define STARTCONVO      0x44  // Tells device to take a temperature reading and put it on the scratchpad
#define COPYSCRATCH     0x48  // Copy scratchpad to EEPROM
#define READSCRATCH     0xBE  // Read from scratchpad
#define WRITESCRATCH    0x4E  // Write to scratchpad
#define RECALLSCRATCH   0xB8  // Recall from EEPROM to scratchpad
#define READPOWERSUPPLY 0xB4  // Determine if device needs parasite power
#define ALARMSEARCH     0xEC  // Query bus for devices with an alarm condition

// Scratchpad locations
#define TEMP_LSB        0
#define TEMP_MSB        1
#define HIGH_ALARM_TEMP 2
#define LOW_ALARM_TEMP  3
#define CONFIGURATION   4
#define INTERNAL_BYTE   5
#define COUNT_REMAIN    6
#define COUNT_PER_C     7
#define SCRATCHPAD_CRC  8

bool validFamily(const uint8_t* deviceAddress) {
	switch (deviceAddress[DSROM_FAMILY]) {
	case DS18S20MODEL:
	case DS18B20MODEL:
	case DS1822MODEL:
	case DS1825MODEL:
	case DS28EA00MODEL:
		return true;
	default:
		return false;
	}
}

bool isConnected(const uint8_t* deviceAddress, uint8_t* scratchPad);
uint8_t getResolution(const uint8_t* deviceAddress) {

	// DS1820 and DS18S20 have no resolution configuration register
	if (deviceAddress[DSROM_FAMILY] == DS18S20MODEL)
		return 12;

	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad)) {
		switch (scratchPad[CONFIGURATION]) {
		case TEMP_12_BIT:
			return 12;

		case TEMP_11_BIT:
			return 11;

		case TEMP_10_BIT:
			return 10;

		case TEMP_9_BIT:
			return 9;
		}
	}
	return 0;

}

bool isAllZeros(const uint8_t * const scratchPad) {
	const size_t length = 9;
	for (size_t i = 0; i < length; i++) {
		if (scratchPad[i] != 0) {
			return false;
		}
	}

	return true;
}

bool readScratchPad(const uint8_t* deviceAddress, uint8_t* scratchPad) {

	// send the reset command and fail fast
	int b = reset();
	if (b == 0)
		return false;

	select(deviceAddress);
	write(READSCRATCH);

	// Read all registers in a simple loop
	// byte 0: temperature LSB
	// byte 1: temperature MSB
	// byte 2: high alarm temp
	// byte 3: low alarm temp
	// byte 4: DS18S20: store for crc
	//         DS18B20 & DS1822: configuration register
	// byte 5: internal use & crc
	// byte 6: DS18S20: COUNT_REMAIN
	//         DS18B20 & DS1822: store for crc
	// byte 7: DS18S20: COUNT_PER_C
	//         DS18B20 & DS1822: store for crc
	// byte 8: SCRATCHPAD_CRC
	for (uint8_t i = 0; i < 9; i++) {
		scratchPad[i] = read();
	}

	b = reset();
	return (b == 1);
}

bool isConnected(const uint8_t* deviceAddress, uint8_t* scratchPad) {
	bool b = readScratchPad(deviceAddress, scratchPad);
	return b && !isAllZeros(scratchPad);
}

uint8_t getDeviceCount(void) {
	return devices;
}

void DallasTemperature__begin(void) {
	DeviceAddress deviceAddress;

	reset_search();
	devices = 0; // Reset the number of devices when we enumerate wire devices
	ds18Count = 0; // Reset number of DS18xxx Family devices

	while (search(deviceAddress)) {

		devices++;

		if (validFamily(deviceAddress)) {
			ds18Count++;

			uint8_t b = getResolution(deviceAddress);
			if (b > bitResolution) bitResolution = b;
		}
	}
}

#define MAX_CONVERSION_TIMEOUT		750

bool isConversionComplete() {
	uint8_t b = read_bit();
	return (b == 1);
}

uint16_t millisToWaitForConversion(uint8_t bitResolution) {

	switch (bitResolution) {
	case 9:
		return 94;
	case 10:
		return 188;
	case 11:
		return 375;
	default:
		return 750;
	}

}

void activateExternalPullup() {
	if(useExternalPullup)
		//digitalWrite(pullupPin, LOW);
		gpio_pin_set(dev1,LED1_PIN,LOW);
}

void deactivateExternalPullup() {
	if(useExternalPullup)
		//digitalWrite(pullupPin, HIGH);
		gpio_pin_set(dev1,LED1_PIN,HIGH);
}

void blockTillConversionComplete(uint8_t bitResolution) {

  if (checkForConversion) {
	for (int i = 0; i < MAX_CONVERSION_TIMEOUT; i++) {
		if (isConversionComplete()) break;
		k_msleep(1);
	}
  } else {
    unsigned long delms = millisToWaitForConversion(bitResolution);
    activateExternalPullup();
    k_msleep(delms);
    deactivateExternalPullup();
  }

}

void requestTemperatures() {

	reset();
	skip();
	write(STARTCONVO);

	// ASYNC mode?
	if (!waitForConversion)
		return;
	blockTillConversionComplete(bitResolution);

}

bool getAddress(uint8_t* deviceAddress, uint8_t index) {

	uint8_t depth = 0;

	reset_search();

	while (depth <= index && search(deviceAddress)) {
		if (depth == index)
			return true;
		depth++;
	}

	return false;

}

int16_t calculateTemperature(const uint8_t* deviceAddress,
		uint8_t* scratchPad) {

	int16_t fpTemperature = (((int16_t) scratchPad[TEMP_MSB]) << 11)
			| (((int16_t) scratchPad[TEMP_LSB]) << 3);

	/*
	 DS1820 and DS18S20 have a 9-bit temperature register.
	 Resolutions greater than 9-bit can be calculated using the data from
	 the temperature, and COUNT REMAIN and COUNT PER °C registers in the
	 scratchpad.  The resolution of the calculation depends on the model.
	 While the COUNT PER °C register is hard-wired to 16 (10h) in a
	 DS18S20, it changes with temperature in DS1820.
	 After reading the scratchpad, the TEMP_READ value is obtained by
	 truncating the 0.5°C bit (bit 0) from the temperature data. The
	 extended resolution temperature can then be calculated using the
	 following equation:
	                                  COUNT_PER_C - COUNT_REMAIN
	 TEMPERATURE = TEMP_READ - 0.25 + --------------------------
	                                         COUNT_PER_C
	 Hagai Shatz simplified this to integer arithmetic for a 12 bits
	 value for a DS18S20, and James Cameron added legacy DS1820 support.
	 See - http://myarduinotoy.blogspot.co.uk/2013/02/12bit-result-from-ds18s20.html
	 */

	if ((deviceAddress[DSROM_FAMILY] == DS18S20MODEL) && (scratchPad[COUNT_PER_C] != 0)) {
		fpTemperature = ((fpTemperature & 0xfff0) << 3) - 32
				+ (((scratchPad[COUNT_PER_C] - scratchPad[COUNT_REMAIN]) << 7)
						/ scratchPad[COUNT_PER_C]);
	}

	return fpTemperature;
}

// returns temperature in 1/128 degrees C or DEVICE_DISCONNECTED_RAW if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_RAW is defined in
// DallasTemperature.h. It is a large negative number outside the
// operating range of the device
int16_t getTemp(const uint8_t* deviceAddress) {

	ScratchPad scratchPad;
	if (isConnected(deviceAddress, scratchPad))
		return calculateTemperature(deviceAddress, scratchPad);
	return DEVICE_DISCONNECTED_RAW;

}

float rawToCelsius(int16_t raw) {

	if (raw <= DEVICE_DISCONNECTED_RAW)
		return DEVICE_DISCONNECTED_C;
	// C = RAW/128
	return (float) raw * 0.0078125f;

}


// returns temperature in degrees C or DEVICE_DISCONNECTED_C if the
// device's scratch pad cannot be read successfully.
// the numeric value of DEVICE_DISCONNECTED_C is defined in
// DallasTemperature.h. It is a large negative number outside the
// operating range of the device
float getTempC(const uint8_t* deviceAddress) {
	return rawToCelsius(getTemp(deviceAddress));
}

float getTempCByIndex(uint8_t deviceIndex) {

	DeviceAddress deviceAddress;
	if (!getAddress(deviceAddress, deviceIndex)) {
		return DEVICE_DISCONNECTED_C;
	}
	return getTempC((uint8_t*) deviceAddress);
}

void main(void)
{
	
	int ret;
	enable_uart_console();
	configure_device();

	dev1 = device_get_binding(LED1);
	if (dev1 == NULL) {
		return;
	}

	ret = gpio_pin_configure(dev1, LED1_PIN, GPIO_OUTPUT);
	if (ret < 0) {
		return;
	}

	{
		int n_devices;
		sht31_t sht31_data;
		DallasTemperature__begin();		

		n_devices = getDeviceCount();

		while (1) {
			read_temp_hum(&sht31_data);
			print_data_sht(&sht31_data);
			requestTemperatures();
			printf("Total devices: %d\n", n_devices);
			for (int i = 0; i < n_devices; i++) {
				printf("sensor %d: %f\n", i, getTempCByIndex(i));
			}
			k_msleep(1000);
		}
	}
}
