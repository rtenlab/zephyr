#ifndef DALLAS_H
#define DALLAS_H

#include "Onewire.h"


typedef uint8_t DeviceAddress[8];
typedef uint8_t ScratchPad[9];

// parasite power on or off
extern bool parasite;

// external pullup
extern bool useExternalPullup;
extern uint8_t pullupPin;

// used to determine the delay amount needed to allow for the
// temperature conversion to take place
extern uint8_t bitResolution;

// used to requestTemperature with or without delay
extern bool waitForConversion;

// used to requestTemperature to dynamically check if a conversion is complete
extern bool checkForConversion;

// used to determine if values will be saved from scratchpad to EEPROM on every scratchpad write
extern bool autoSaveScratchPad;

// count of devices on the bus
extern uint8_t devices;

// count of DS18xxx Family devices on bus
extern uint8_t ds18Count;

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

#define MAX_CONVERSION_TIMEOUT 750

bool validFamily(const uint8_t* deviceAddress);

bool isConnected(const uint8_t* deviceAddress, uint8_t* scratchPad);

uint8_t getResolution(const uint8_t* deviceAddress);

bool isAllZeros(const uint8_t * const scratchPad);

bool readScratchPad(const uint8_t* deviceAddress, uint8_t* scratchPad);

bool isConnected(const uint8_t* deviceAddress, uint8_t* scratchPad);

void DallasTemperature_begin(void);

bool isConversionComplete(void);

uint16_t millisToWaitForConversion(uint8_t bitResolution);

void activateExternalPullup(void);

void deactivateExternalPullup(void);

void blockTillConversionComplete(uint8_t bitResolution);

void requestTemperatures(void);

bool getAddress(uint8_t* deviceAddress, uint8_t index);

int16_t calculateTemperature(const uint8_t* deviceAddress, uint8_t* scratchPad);

int16_t getTemp(const uint8_t* deviceAddress);

float rawToCelsius(int16_t raw);

float getTempC(const uint8_t* deviceAddress);

float getTempCByIndex(uint8_t deviceIndex);

uint8_t getDeviceCount(void);
#endif