#ifndef ONEWIRE_H
#define ONEWIRE_H


#include <zephyr.h>
#include <device.h>
#include <devicetree.h>
#include <stdio.h>
#include <drivers/gpio.h>

/* The devicetree node identifier for the "led2" alias. */
#define DS_NODE DT_ALIAS(ds0)

#if DT_NODE_HAS_STATUS(DS_NODE, okay)
#define DS_SENSOR	        DT_GPIO_LABEL(DS_NODE, gpios)
#define DS_SENSOR_PIN		DT_GPIO_PIN(DS_NODE, gpios)
#define DS_SENSOR_FLAGS	    DT_GPIO_FLAGS(DS_NODE, gpios)
#else
/* A build error here means your board isn't set up to blink an LED. */
#error "Unsupported board: led1 devicetree alias is not defined"
#define DS_SENSOR	        ""
#define DS_SENSOR_PIN	0
#define DS_SENSOR_FLAGS	    0
#endif
#define LOW 0
#define HIGH 1


#define ONEWIRE_SEARCH 1
#define ONEWIRE_CRC 0

// global search state
extern unsigned char ROM_NO[8];
extern uint8_t LastDiscrepancy;
extern uint8_t LastFamilyDiscrepancy;
extern bool LastDeviceFlag;

// /**
//  * @brief  uses k_busy_wait() function to busy wait for inputed microseconds
//  * @param  secs: microseconds for which we need to wait.
//  * @retval None
//  */
// static inline void udelay(int);
/**
 * @brief: Send a reset signal to the DS18B20 sensor. 
 *          Pull the bus low for 480 microseconds and pull up for 70 microseconds.
 * @retval: Checks if the any sensor on the bus acknowledge. 
 *          Returns 1 if acknowledge by the sensor on the bus, else returns 0.
 */
uint8_t reset(void);

/**
 * @brief  Read a single bit from the bus. 
 *          PUll sht ebus low for minimum 1 us and let it float for 10 us. read the input after the sequence.
 * @retval Returns a bit read from the bus.
 */
uint8_t read_bit(void);

/**
 * @brief  Read a byte from the bus.
 *          Use read_bit() functions 8 times and start filling a 8-bit variable from LSB to MSB.
 * @retval Returns a 1 Byte variable obtained from the bus.
 */
uint8_t read(void);

/**
 * @brief  Writes a bit to the bus.
 * @param  1 bit required to be written to the bus.
 * @retval None
 */
void write_bit(uint8_t);

/**
 * @brief  Write 1 Byte (8-bits) to the bus.
 * @param  v: 1 byte that needs to be written to the bus.
 * @retval None
 */
void write(uint8_t);

/**
 * @brief  Look for the next device.
 * @param  Address required to search for the next device on the bus.
 * @retval Returns 1 if a new address has been returned. A zero might mean that the bus is shorted, there are
     no devices, or you have already retrieved all of them.
 */
bool search(uint8_t *);

/**
 * @brief Issue a 1-Wire rom select command, you do the reset first.  
 * @param  uint8_t[8]: The ROM code that needs to be selected
 * @retval None
 */
void select(const uint8_t*);

/**
 * @brief  Clear the search state so that if will start from the beginning again.
 * @retval None
 */
void reset_search(void);

/**
 * @brief  Issue a 1-Wire rom skip command, to address all Sensors on bus.
 * @retval None
 */
void skip(void);

#endif

