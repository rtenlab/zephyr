/*
 * Copyright (c) 2018-2019 Peter Bigot Consulting, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef APPLICATION_BATTERY_H_
#define APPLICATION_BATTERY_H_


#define BT_UUID_VOLTAGE_VAL 0X2ae1
#define BT_UUID_VOLTAGE\
 	BT_UUID_DECLARE_16(BT_UUID_VOLTAGE_VAL)
	 
/** A discharge curve specific to the power source. */
// static const struct battery_level_point levels[] = {
// #if DT_NODE_HAS_PROP(DT_INST(0, voltage_divider), io_channels)
// 	/* "Curve" here eyeballed from captured data for the [Adafruit
// 	 * 3.7v 2000 mAh](https://www.adafruit.com/product/2011) LIPO
// 	 * under full load that started with a charge of 3.96 V and
// 	 * dropped about linearly to 3.58 V over 15 hours.  It then
// 	 * dropped rapidly to 3.10 V over one hour, at which point it
// 	 * stopped transmitting.
// 	 *
// 	 * Based on eyeball comparisons we'll say that 15/16 of life
// 	 * goes between 3.95 and 3.55 V, and 1/16 goes between 3.55 V
// 	 * and 3.1 V.
// 	 */

// 	{ 10000, 3950 },
// 	{ 625, 3550 },
// 	{ 0, 3100 },
// #else
// 	/* Linear from maximum voltage to minimum voltage. */
// 	{ 10000, 3600 },
// 	{ 0, 1700 },
// #endif
// };

/** Enable or disable measurement of the battery voltage.
 *
 * @param enable true to enable, false to disable
 *
 * @return zero on success, or a negative error code.
 */
int battery_measure_enable(bool enable);

/** Measure the battery voltage.
 *
 * @return the battery voltage in millivolts, or a negative error
 * code.
 */
float battery_sample(void);

/** A point in a battery discharge curve sequence.
 *
 * A discharge curve is defined as a sequence of these points, where
 * the first point has #lvl_pptt set to 10000 and the last point has
 * #lvl_pptt set to zero.  Both #lvl_pptt and #lvl_mV should be
 * monotonic decreasing within the sequence.
 */
struct battery_level_point {
	/** Remaining life at #lvl_mV. */
	uint16_t lvl_pptt;

	/** Battery voltage at #lvl_pptt remaining life. */
	uint16_t lvl_mV;
};

/** Calculate the estimated battery level based on a measured voltage.
 *
 * @param batt_mV a measured battery voltage level.
 *
 * @param curve the discharge curve for the type of battery installed
 * on the system.
 *
 * @return the estimated remaining capacity in parts per ten
 * thousand.
 */
unsigned int battery_level_pptt(unsigned int batt_mV,
				const struct battery_level_point *curve);

#endif /* APPLICATION_BATTERY_H_ */
