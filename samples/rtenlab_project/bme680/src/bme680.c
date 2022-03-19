#include "bme680.h"
#include "uart_i2c.h"

/**
 * @brief  Get the ID of the sensor and check if it matches the pre-defined chip ID or not.
 * @note   
 * @retval True -> if the register read was successfull and the chip id matches. False otherwise.
 */
bool bme680_getid(void){
    uint8_t id;
    uint8_t ret = i2c_reg_read_byte(dev_i2c, BME680_Addr, BME680_ID, &id);
    if(ret!=0){
        printk("Error Reading ID of the BME 680 Sensor\n");
        return false;
    }
    printk("Chip ID of BME 680 is: 0x%x\n", id);
    // 0x61 is the chip ID for BME680.
    if(id!=0x61)
        return false;
    return true;    
}