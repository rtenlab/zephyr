#include "bme680.h"
// #define FPU_EN
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
    #ifdef BME680_DEBUG
    printk("Chip ID of BME 680 is: 0x%x\n", id);
    #endif
    // 0x61 is the chip ID for BME680.
    if(id!=0x61)
        return false;
    return true;    
}

bool bme680_get_gas_calib_data(bme680_gas_par_t* calib){
    uint8_t temp=0;
    uint8_t ret = i2c_reg_read_byte(dev_i2c, BME680_Addr, BME680_par_g1, &temp);
    calib->para1 = (int8_t)temp;
    temp=0;
    if(ret!=0){
        printk("Error while reading the first calibration parameter!!!\n Exiting....\n");
        return false;
    } 
    uint8_t buf[2] = {0,0};
    ret = i2c_burst_read(dev_i2c, BME680_Addr, BME680_par_g2_LSB, &buf[0], 2);
    if(ret!=0){
        printk("Error while reading the second calibration parameter!!!\n Exiting....\n");
        return false;
    }
    calib->para2 = (uint16_t)((buf[1]<<8) | (buf[0]));
    
    ret = i2c_reg_read_byte(dev_i2c, BME680_Addr, BME680_par_g3, &temp);
    calib->para3 = (int8_t)temp;
    if(ret!=0){
        printk("Error while reading the third calibration parameter!!!\n Exiting....\n");
        return false;
    }
    temp=0;
    ret = i2c_reg_read_byte(dev_i2c, BME680_Addr, BME680_res_heat_range, &temp);
    if(ret!=0){
        printk("Error while reading the third calibration parameter!!!\n Exiting....\n");
        return false;
    }
    printk("Temp for res_heat_range: %d\n", temp);
    // Just get the 4th and 5th bit and right shift it 4 times to start with the zero.
    calib->res_heat_range = ((temp & BME680_res_heat_range_mask) / 16);
    
    temp=0;
    ret = i2c_reg_read_byte(dev_i2c, BME680_Addr, BME680_res_heat_val, &temp);
    if(ret!=0){
        printk("Error while reading the third calibration parameter!!!\n Exiting....\n");
        return false;
    }
    calib->res_heat_val= (int8_t)temp;

    temp=0;
    ret = i2c_reg_read_byte(dev_i2c, BME680_Addr, BME680_range_sw_err, &temp);
    if(ret!=0){
        printk("Error while reading the third calibration parameter!!!\n Exiting....\n");
        return false;
    }
    #ifdef BME680_DEBUG
    printk("Temp for res_range_err: %d\n", temp);
    #endif
    calib->range_sw_err= ((int8_t)temp & (int8_t)BME680_RSERROR_MSK) / 16;
    calib->amb_temp = 25;
    return true;
}

uint8_t bme680_calc_gas_wait(uint16_t dur)
{
    uint8_t factor = 0;
    uint8_t durval;

    if (dur >= 0xfc0)
    {
        durval = 0xff; /* Max duration*/
    }
    else
    {
        while (dur > 0x3F)
        {
            dur = dur / 4;
            factor += 1;
        }

        durval = (uint8_t)(dur + (factor * 64));
    }

    return durval;
}


bool bme680_set_heater_conf(uint16_t heaterTemp, uint16_t heaterTime, bme680_gas_par_t* calib){
    // If the heating temperature and time is invalid return false.
    if((heaterTemp==0) || (heaterTime==0)){
        printk("Invalid Temp. or Time\n");
        return false;
    }
    // Setting the nbconv and making the run_gas bit to 1 so that it will start measuring the gas values.
    uint8_t nb_conv_msk = 0x10;
    uint8_t ret = i2c_reg_write_byte(dev_i2c, BME680_Addr, BME680_Ctrl_gas_1, nb_conv_msk);
    if(ret!=0){
        printk("Problem in setting the nb_conv in bme680_set_heater_conf Function!!!\n");
        return false;
    }
    uint8_t res_heat = bme680_calculate_res_heat(heaterTemp, calib);
    uint8_t gas_wait = bme680_calc_gas_wait(heaterTime);
    // Setting the gas wait by changing the actual value using the function "bme_calc_gas_wait".
    gas_wait &= BME680_Gas_Wait0_msk;
    ret = i2c_reg_write_byte(dev_i2c, BME680_Addr, BME680_Gas_Wait0, gas_wait);
    if(ret!=0){
        printk("Problem in setting the gas_Wait in bme680_set_heater_conf Function!!!\n");
        return false;
    }
    // Setting the resistance heating by changing the actual value using the function "bme680_calculate_res_heat".
    ret = i2c_reg_write_byte(dev_i2c, BME680_Addr, BME680_Res_Heat0, res_heat);
    if(ret!=0){
        printk("Problem in setting the res_heat in bme680_set_heater_conf Function!!!\n");
        return false;
    }
    // If no errors, return true.
    return true;
}

/**
 * @brief  set the current power mode of the sensor
 * @note   
 * @param  mode: 0-> Sleep mode. 1->Forced Mode
 * @retval TRUE: if the power mode was set successfully; FALSE: if something wrong with i2c_reg_read_byte 
 */
bool bme680_set_power_mode(uint8_t mode){
    uint8_t ret;
    if(mode==1){
        ret = i2c_reg_write_byte(dev_i2c, BME680_Addr, BME680_Ctrl_meas, mode);
        if(ret!=0){
            printk("Problem settting the mode to 1\n");
            return false;
        }
    }
    else{
        mode=0;
        ret = i2c_reg_write_byte(dev_i2c, BME680_Addr, BME680_Ctrl_meas, mode);
        if(ret!=0){
            printk("Problem settting the mode to 0\n");
            return false;
        }
    }
    return true;
}

bool bme680_get_raw_gas_data(bme680_gas_data_t* gasdata){
    uint8_t ret,  buffer[2] = {0};
    ret = i2c_burst_read(dev_i2c, BME680_Addr, BME680_Gas_r_msb, &buffer[0], 2);
    if(ret!=0){
        printk("Problem while reading the MSB of the Gas ADC value in function 'bme680_get_gas_value'\n");
        return false;
    }
    gasdata->adc_gas_res = (uint16_t)((((uint32_t)buffer[0])<<2) | (((uint32_t)buffer[1]&0xC0)>>6));
    // gasdata->adc_gas_res_high = buffer[0];
    // gasdata->adc_gas_res_low = (buffer[1]&0xC0)>>6;
    gasdata->gas_range = (buffer[1]&0x0F);
    gasdata->gas_valid_bit = (buffer[1]&0x20);
    gasdata->heater_stability_bit = (buffer[1]&0x10);    
    #ifdef DEBUG
    printk("Gas_valid: %d\n Heater_Stability: %d\n", gasdata->gas_valid_bit, gasdata->heater_stability_bit);
    #endif
    return true;
}


bool bme680_check_new_data(void){
    uint8_t ret, flag;
    ret = i2c_reg_read_byte(dev_i2c, BME680_Addr, BME680_Eas_status_0, &flag);
    if(ret!=0){
        printk("Problem while reading the EAS Status register in function 'bme680_check_new_data'\n");
        return false;
    }
    #ifdef BME680_DEBUG
    printk("EAS_STATUS: %d\n", flag);
    #endif
    return true;
}


#ifdef FPU_EN

float bme680_calc_gas_resistance(bme680_gas_data_t*gasdata,bme680_gas_par_t* calib)
{
	float calc_gas_res;
	float var1 = 0;
	float var2 = 0;
	float var3 = 0;

	const float lookup_k1_range[16] = {
	0.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, -0.8,
	0.0, 0.0, -0.2, -0.5, 0.0, -1.0, 0.0, 0.0};
	const float lookup_k2_range[16] = {
	0.0, 0.0, 0.0, 0.0, 0.1, 0.7, 0.0, -0.8,
	-0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

	var1 = (1340.0f + (5.0f * calib->range_sw_err));
	var2 = (var1) * (1.0f + lookup_k1_range[gasdata->gas_range]/100.0f);
	var3 = 1.0f + (lookup_k2_range[gasdata->gas_range]/100.0f);

	calc_gas_res = 1.0f / (float)(var3 * (0.000000125f) * (float)(1 << gasdata->gas_range) * (((((float)gasdata->adc_gas_res)
		- 512.0f)/var2) + 1.0f));

	return calc_gas_res;
}

/*!
 * @brief This internal API is used to calculate the
 * heater resistance value in float format
 */
float bme680_calculate_res_heat(uint16_t temp, bme680_gas_par_t* calib)
{
	float var1 = 0;
	float var2 = 0;
	float var3 = 0;
	float var4 = 0;
	float var5 = 0;
	float res_heat = 0;

	if (temp > 400) /* Cap temperature */
		temp = 400;

	var1 = (((float)calib->para1 / (16.0f)) + 49.0f);
	var2 = ((((float)calib->para2 / (32768.0f)) * (0.0005f)) + 0.00235f);
	var3 = ((float)calib->para3 / (1024.0f));
	var4 = (var1 * (1.0f + (var2 * (float)temp)));
	var5 = (var4 + (var3 * (float)calib->amb_temp));
	res_heat = (uint8_t)(3.4f * ((var5 * (4 / (4 + (float)calib->res_heat_range)) *
		(1/(1 + ((float) calib->res_heat_val * 0.002f)))) - 25));

	return res_heat;
}

#else

uint8_t bme680_calculate_res_heat(uint16_t temp, bme680_gas_par_t* calib){
    uint8_t heatr_res;
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t heatr_res_x100;

    if (temp > 400) /* Cap temperature */
    {
        temp = 400;
    }

    var1 = (((int32_t)calib->amb_temp * calib->para3) / 1000) * 256;
    var2 = (calib->para1 + 784) * (((((calib->para2 + 154009) * temp * 5) / 100) + 3276800) / 10);
    var3 = var1 + (var2 / 2);
    var4 = (var3 / (calib->res_heat_range + 4));
    var5 = (131 * calib->res_heat_val) + 65536;
    heatr_res_x100 = (int32_t)(((var4 / var5) - 250) * 34);
    heatr_res = (uint8_t)((heatr_res_x100 + 50) / 100);
    return heatr_res;
}


uint32_t bme680_calc_gas_resistance(bme680_gas_data_t* gasdata, bme680_gas_par_t* calibdata)
{
	int64_t var1;
	uint64_t var2;
	int64_t var3;
	uint32_t calc_gas_res;
	/**Look up table 1 for the possible gas range values */
	uint32_t lookupTable1[16] = { UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2147483647),
		UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2130303777),
		UINT32_C(2147483647), UINT32_C(2147483647), UINT32_C(2143188679), UINT32_C(2136746228),
		UINT32_C(2147483647), UINT32_C(2126008810), UINT32_C(2147483647), UINT32_C(2147483647) };
	/**Look up table 2 for the possible gas range values */
	uint32_t lookupTable2[16] = { UINT32_C(4096000000), UINT32_C(2048000000), UINT32_C(1024000000), UINT32_C(512000000),
		UINT32_C(255744255), UINT32_C(127110228), UINT32_C(64000000), UINT32_C(32258064), UINT32_C(16016016),
		UINT32_C(8000000), UINT32_C(4000000), UINT32_C(2000000), UINT32_C(1000000), UINT32_C(500000),
		UINT32_C(250000), UINT32_C(125000) };

	var1 = (int64_t) ((1340 + (5 * (int64_t) calibdata->range_sw_err)) *
		((int64_t) lookupTable1[gasdata->gas_range])) >> 16;
	var2 = (((int64_t) ((int64_t) gasdata->adc_gas_res << 15) - (int64_t) (16777216)) + var1);
	var3 = (((int64_t) lookupTable2[gasdata->gas_range] * (int64_t) var1) >> 9);
	calc_gas_res = (uint32_t) ((var3 + ((int64_t) var2 >> 1)) / (int64_t) var2);


	return calc_gas_res;
}
#endif
