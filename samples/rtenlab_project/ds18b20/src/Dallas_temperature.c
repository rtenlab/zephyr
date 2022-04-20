#include "Dallas_temperature.h"

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


extern const struct device *dev_ds18b20;
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

void DallasTemperature_begin(void) {
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



bool isConversionComplete(void) {
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

void activateExternalPullup(void) {
	if(useExternalPullup)
		//digitalWrite(pullupPin, LOW);
		gpio_pin_set(dev_ds18b20,LED1_PIN,LOW);
}

void deactivateExternalPullup(void) {
	if(useExternalPullup)
		//digitalWrite(pullupPin, HIGH);
		gpio_pin_set(dev_ds18b20,LED1_PIN,HIGH);
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

void requestTemperatures(void) {

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
