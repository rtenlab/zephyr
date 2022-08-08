/* Multi-threading Code for HoneyBee Monitoring System */

/*
 * Author: Dev Joshi
 * Email Id: djosh011@ucr.edu
 * 
 */

// #include "uart_i2c.c"
#include <zephyr.h>
#include <sys/printk.h>
#include <errno.h>

#include"setup_defines.h"
#include "ble_setup.c"



volatile uint8_t stable_temperature = 0;
volatile bool is_period_changed = false;
// Data Acquisition period will be 20 mins under normal conditions.
volatile uint8_t normal_producer_timer_period = (1*60);
// Data Sending period will be 20 mins under normal conditions.
volatile uint8_t normal_consumer_timer_period = (1*60);
// Data Acquisition period will be 1sec if abnormal temperatures are detected.
volatile uint8_t abnormal_producer_timer_period = (0.1*60);
// Data Sending period will also be 1 mins if abnormal temperatures are detected.
volatile uint8_t abnormal_consumer_timer_period = (0.1*60);


/*
 * The hello world demo has two threads that utilize semaphores and sleeping
 * to take turns printing a greeting message at a controlled rate. The demo
 * shows both the static and dynamic approaches for spawning a thread; a real
 * world application would likely use the static approach for both threads.
 */

#define PIN_THREADS (IS_ENABLED(CONFIG_SMP)		  \
		     && IS_ENABLED(CONFIG_SCHED_CPU_MASK) \
		     && (CONFIG_MP_NUM_CPUS > 1))



/* scheduling priority used by each thread */
#define PRIORITY 7

bool abnormal_temperature_detected = false;
struct k_mutex mymutex, tempvalue;

#ifdef SHT31
/**
 * @brief check the temperature value of the current data acquitision of sht31. If the value is above certain limit, raise a flag. 
 * 
 * @param sht31tempPtr 
 * @return true 
 * @return false 
 */
static inline void sht31_check_temperature(sht31_t* sht31tempPtr){
	if(sht31tempPtr->temp >=SWARMING_TEMP_THRESHOLD){
		/**
		 * @brief abnormal temp. detected. 
		 * TODO: raise the flag and change the period.
		 */
		// making sure that the abnormal temperature flag is clear before setting it. 
		if(!abnormal_temperature_detected){
			abnormal_temperature_detected=true;
			//change the period.
			k_timer_start(&producer_timer, K_SECONDS(abnormal_producer_timer_period), K_SECONDS(abnormal_producer_timer_period));
			k_timer_start(&consumer_timer, K_SECONDS(abnormal_consumer_timer_period), K_SECONDS(abnormal_consumer_timer_period));
		}// end inner if.		
	}// end outer if.
}// end function.


void sht31(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	/* invoke routine to ping-pong hello messages with threadB */
	struct k_thread *current_thread;
	current_thread = k_current_get();
	ble_data_t sht31_local_data;
	sht31_local_data.type = SENSOR_SHT31;
	static uint32_t prev_timestamp, curr_timestamp;
	static uint8_t called_counter=0;
	while(1){
		//Data acquisition start.
		k_mutex_lock(&mymutex, K_FOREVER);
		curr_timestamp = k_uptime_get_32();
		sht31_local_data.period = (curr_timestamp-prev_timestamp)/1000.00;
		read_temp_hum(&sht31_local_data.sht31_data);
		prev_timestamp=curr_timestamp;
		printk("[%f] Hello from %s\n", sht31_local_data.period,k_thread_name_get(current_thread));
		k_msgq_put(&my_msgq, &sht31_local_data, K_FOREVER);
		printk("Done putting the sht data in the message queue\n");
		k_mutex_unlock(&mymutex);
		//Data Acquisition end.


		// Abnormal activity check start.
		if(abnormal_temperature_detected==true){
			if(++called_counter >= 60){
				// if sht is called for 60 times since the abnormal activity detected, clear the abnormal activity flag and counter value to 0;
				called_counter = 0;
				abnormal_temperature_detected=false;

				// changing the period to normal temperature.
				k_timer_start(&producer_timer, K_SECONDS(normal_producer_timer_period), K_SECONDS(normal_producer_timer_period));
				k_timer_start(&consumer_timer, K_SECONDS(normal_consumer_timer_period), K_SECONDS(normal_consumer_timer_period));
			}
			else{
				// Nothing to do. Wait until the counter reaches value 60.
				printk("abnormal_temperature_flag: [%d]; counter: [%d]\n", abnormal_temperature_detected, called_counter);
			}// end of else
		}// end if

		// if abnormal activity is not measured in last 60 mins.
		else{
			// Que: check if putting in the queue destroy this memory or it just copies the data to the queue.
			// Ans: Data is copied and won't be changed by the msq_put.
			sht31_check_temperature(&sht31_local_data.sht31_data);
			printf("TEST: This should always be called with counter: %d\n", called_counter);

			// period will be changed to 1 minute in the above function call if current sht temperature is abnormal.
		}
		// abnormal activity check end.
		printk("SHT Ended!!!\n");
		
		k_sleep(K_FOREVER);
	}
	return;
}
#endif

#ifdef BMP280

void bmp280(void* dummy1, void* dummy2, void* dummy3){
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	read_calibration_registers();
	struct k_thread *current_thread;
	current_thread = k_current_get();
	ble_data_t bmp280_local_data;
	bmp280_local_data.type = SENSOR_BMP280;
	static uint32_t prev_timestamp, curr_timestamp;
	while(1){
		k_mutex_lock(&mymutex, K_FOREVER);
		curr_timestamp=k_uptime_get_32();
		bmp280_local_data.period = (curr_timestamp-prev_timestamp)/1000.00;
		bmp_read_press_temp_data(&bmp280_local_data.bmp280_data);
		// bmp280_local_data.period = k_cycle_get_32()
		prev_timestamp = curr_timestamp;
		printk("[%f] Hello from %s\n", bmp280_local_data.period,k_thread_name_get(current_thread));
		k_msgq_put(&my_msgq, &bmp280_local_data, K_FOREVER);
		if(bmp280_local_data.bmp280_data.temperature>=SWARMING_TEMP_THRESHOLD){
			k_mutex_lock(&tempvalue, K_FOREVER);
			stable_temperature |= (1<<BMP280_POS);
			k_mutex_unlock(&tempvalue);
		}
		printk("BMP Ended!!!\n");
		k_mutex_unlock(&mymutex);
		k_sleep(K_FOREVER);
	}
}
#endif



#ifdef APDS9960
void apds9960(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	struct k_thread *current_thread;
	// apds9960_t threadC_apds9960;
	current_thread = k_current_get();
	// Write something to start another sensor.
	ble_data_t apds_local_data;
	apds_local_data.type = SENSOR_APDS9960;
	static uint32_t prev_timestamp, curr_timestamp;
	while(1){
		k_mutex_lock(&mymutex, K_FOREVER);
		curr_timestamp = k_uptime_get_32();		
		apds_local_data.period = (curr_timestamp-prev_timestamp)/1000.00;
		read_proximity_data(&apds_local_data.apds_cls_data);
		read_als_data(&apds_local_data.apds_cls_data);
		// This is how it will change the period of the timer. 
		prev_timestamp = curr_timestamp;
		printk("[%f] Hello from %s\n", apds_local_data.period,k_thread_name_get(current_thread));
		k_msgq_put(&my_msgq, &apds_local_data, K_FOREVER);
		printk("APDS Ended!!!\n");
		k_mutex_unlock(&mymutex);
		k_sleep(K_FOREVER);
	}	
	return;
}
#endif



#ifdef LSM6DS33

void lsm6ds33(void* dummy1, void* dummy2, void* dummy3){
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	struct k_thread* current_thread;
	current_thread = k_current_get();
	ble_data_t lsm6ds33_local_data;
	lsm6ds33_local_data.type = SENSOR_LSM6DS33;
	static uint32_t prev_timestamp, curr_timestamp;
	float accelX, accelY, accelZ;	
	float totalX, totalY, totalZ;
	uint8_t count;
	// Initialize the sensor by setting necessary parameters for gyroscope and accelerometer
	accel_set_power_mode(ACCEL_LOW_POWER_MODE);
	gyro_set_power_mode(GYRO_LOW_POWER_MODE);
	//  Check if the FIFO_FULL Flag is set or not.
	while(1){
		// Mutex is locked for very long time. Need to make sure how to do this.
		printk("LSM: Waiting for Mutex\n");
		// while (1) {
		// 	// k_mutex_lock(&mymutex, K_FOREVER);
		// 	// if (lsm6ds33_fifo_status() & FIFO_FULL) == 0) {
		// 	// 	k_mutex_unlock(&mymutex);
		// 	// 	sleep(xx); // for some time or
		// 	// 	or k_yield(); // relinquish CPU
		// 	// }
		// 	// else break;
		// }
		k_mutex_lock(&mymutex, K_FOREVER);
		curr_timestamp = k_uptime_get_32();
		lsm6ds33_local_data.period = (curr_timestamp-prev_timestamp)/1000.00;
		prev_timestamp = curr_timestamp;
		printk("[%f] Hello from %s\n", lsm6ds33_local_data.period,k_thread_name_get(current_thread));
		printk("Waiting for the FIFO mode to get the data...\n");
		while((lsm6ds33_fifo_status() & FIFO_FULL) == 0) { };
		count=0, totalX=0, totalY=0, totalZ=0;
		while( (lsm6ds33_fifo_status()& FIFO_EMPTY) == 0){
			accelX = lsm6ds33_fifo_get_accel_data(lsm6ds33_fifo_read());
			totalX+=accelX;
			accelY = lsm6ds33_fifo_get_accel_data(lsm6ds33_fifo_read());
			totalY+=accelY;
			accelZ = lsm6ds33_fifo_get_accel_data(lsm6ds33_fifo_read());
			totalZ+=accelZ;
			count++;
		}
		printk("Average X: %f\t Y: %f\t Z: %f\n", totalX/count, totalY/count, totalZ/count);
		lsm6ds33_local_data.lsm6ds33_data.accelX = totalX/count;
		lsm6ds33_local_data.lsm6ds33_data.accelY = totalY/count;
		lsm6ds33_local_data.lsm6ds33_data.accelZ = totalZ/count;
		k_msgq_put(&my_msgq, &lsm6ds33_local_data, K_FOREVER);
		lsm6ds33_fifo_change_mode(0);
		k_sleep(K_FOREVER);
		k_mutex_unlock(&mymutex);
		printk("LSM6DS33 Finished!!!\n");
		lsm6ds33_fifo_change_mode(1);
	}
}
#endif

char* enum_to_string(ble_data_t* data){
	if(data->type == SENSOR_APDS9960){
		return "APDS9960";
	}
	else if(data->type == SENSOR_SHT31){
		return "SHT31";
	}
	else if(data->type == SENSOR_BMP280){
		return "BMP280";
	}
	else if(data->type == SENSOR_LSM6DS33){
		return "LSM6DS33";
	}
	
	return "Not Valid";
}



void consumer_thread(void* dummy1, void* dummy2, void* dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	ble_data_t consumer_local_data;
	uint32_t time=0;
	printk("Consumer Thread: Entered for the first time!!!\n");
	while(1){
		uint8_t num_used = k_msgq_num_used_get(&my_msgq);
		for(int i=0; i<num_used; i++){
			int8_t ret = k_msgq_get(&my_msgq, &consumer_local_data, K_MSEC(100));
			if(ret!=0){
				printk("Couldn't find data\n");
			}
			time = k_cycle_get_32();
			if(consumer_local_data.type == SENSOR_APDS9960){
				printk("[Current: %d\t Data_time_stamp: %f] Data type: %s\t Prox: %d\t Clear: %d\t Red: %d\t Blue: %d\t Green: %d\n",
													time,
													consumer_local_data.period,
													enum_to_string(&consumer_local_data),
													consumer_local_data.apds_cls_data.proximity, 
														consumer_local_data.apds_cls_data.clear, 
														consumer_local_data.apds_cls_data.red,
														consumer_local_data.apds_cls_data.blue, 
														consumer_local_data.apds_cls_data.green);
#ifdef BLE
				// Add code to send it through BLE.
				bt_gatt_notify(NULL, &ess_svc.attrs[APDS_BLE_HANDLE], &consumer_local_data.apds_cls_data.clear, sizeof(consumer_local_data.apds_cls_data.clear));
#endif
			}
			else if(consumer_local_data.type == SENSOR_SHT31){
				printk("[Current: %d\t Data_time_stamp: %f] Data type: %s\t Temp: %f\t Humi: %f\n",
													time, 
													consumer_local_data.period,
													enum_to_string(&consumer_local_data),
														consumer_local_data.sht31_data.temp, 
														consumer_local_data.sht31_data.humidity);
				int16_t temp_value, hum_value;
				temp_value = (uint16_t)((consumer_local_data.sht31_data.temp)*100);
				hum_value = (uint16_t)((consumer_local_data.sht31_data.humidity)*100);
#ifdef BLE				
				bt_gatt_notify(NULL, &ess_svc.attrs[SHT_TEMP_BLE_HANDLE], &temp_value, sizeof(temp_value));
				delay(100);
				bt_gatt_notify(NULL, &ess_svc.attrs[SHT_HUM_BLE_HANDLE], &hum_value, sizeof(hum_value));
#endif
			}

			else if(consumer_local_data.type == SENSOR_BMP280){
				printk("[Current: %d\t Data_time_stamp: %f] Data type: %s\t Temp: %f\t Press: %f\n",
													time, 
													consumer_local_data.period,
													enum_to_string(&consumer_local_data),
														consumer_local_data.bmp280_data.temperature, 
														consumer_local_data.bmp280_data.pressure);
				static uint16_t temperature;
				temperature = (uint16_t)((consumer_local_data.bmp280_data.temperature)*100);
				static uint32_t pressure;
				pressure = (uint32_t)((consumer_local_data.bmp280_data.pressure)*10);
#ifdef BLE
				bt_gatt_notify(NULL, &ess_svc.attrs[BMP_TEMP_BLE_HANDLE], &temperature, sizeof(temperature));
				delay(100);
				bt_gatt_notify(NULL, &ess_svc.attrs[BMP_PRESS_BLE_HANDLE], &pressure, sizeof(pressure));
#endif 
			}

			else if(consumer_local_data.type == SENSOR_LSM6DS33){
				printk("[Current: %d\t Data_time_stamp: %f] Data type: %s\t AccelX: %f\t AccelY: %f\t AccelZ: %f\n",
													time, 
													consumer_local_data.period,
													enum_to_string(&consumer_local_data),
														consumer_local_data.lsm6ds33_data.accelX, 
														consumer_local_data.lsm6ds33_data.accelY,
														consumer_local_data.lsm6ds33_data.accelZ);
				static int16_t finalX, finalY, finalZ;
				finalX = (int16_t)((consumer_local_data.lsm6ds33_data.accelX)*100+32768);
				finalY = (int16_t)((consumer_local_data.lsm6ds33_data.accelY)*100+32768);
				finalZ = (int16_t)((consumer_local_data.lsm6ds33_data.accelZ)*100+32768);
#ifdef BLE
				bt_gatt_notify(NULL, &ess_svc.attrs[LSM_ACCELX_BLE_HANDLE], &finalX, sizeof(finalX));
				delay(100);
				bt_gatt_notify(NULL, &ess_svc.attrs[LSM_ACCELY_BLE_HANDLE], &finalY, sizeof(finalY));
				delay(100);
				bt_gatt_notify(NULL, &ess_svc.attrs[LSM_ACCELZ_BLE_HANDLE], &finalZ, sizeof(finalZ));
#endif
			}
			
		}// End of if.
		k_sleep(K_FOREVER);

	}

}


// void temperature_checker_thread(void* dummy1, void* dummy2, void* dummy3){
// 	ARG_UNUSED(dummy1);
// 	ARG_UNUSED(dummy2);
// 	ARG_UNUSED(dummy3);
// 	bool is_period_changed = false;
// 	uint8_t mins_since_counter_changed=0;

// }



void main(void)
{
	// This will configure the I2C device so this is necessary.
	configure_device();
#ifdef UART  
	enable_uart_console();
#endif
	k_mutex_init(&mymutex);
	k_mutex_init(&tempvalue);
	printk("Number of CPUS: %d\n", CONFIG_MP_NUM_CPUS);

#ifdef BLE
	// code to turn on the bluetooth module and start advertisement.
	int err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return;
	}
	bt_ready();
	bt_conn_cb_register(&conn_callbacks);
#endif

#ifdef SHT31
	k_thread_create(&sht31_thread_data, sht31_stack_area,
			K_THREAD_STACK_SIZEOF(sht31_stack_area),
			sht31, NULL, NULL, NULL,
			PRIORITY, 0, K_NO_WAIT);
	k_thread_name_set(&sht31_thread_data, "SHT_Thread");
#endif
#ifdef APDS9960
	enable_apds_sensor();
	k_thread_create(&apds9960_thread_data, apds9960_stack_area, 
			K_THREAD_STACK_SIZEOF(apds9960_stack_area),
			apds9960, NULL, NULL, NULL, 
			PRIORITY, 0, K_MSEC(20));
	k_thread_name_set(&apds9960_thread_data, "APDS Thread");
#endif

#ifdef BMP280
	k_thread_create(&bmp280_thread_data, bmp280_stack_area, 
		K_THREAD_STACK_SIZEOF(bmp280_stack_area),
		bmp280, NULL, NULL, NULL, 
		PRIORITY, 0, K_MSEC(40));
	k_thread_name_set(&bmp280_thread_data, "BMP Thread");
#endif

#ifdef LSM6DS33
	lsm6ds33_init();
	k_thread_create(&lsm6ds33_thread_data, lsm6ds33_stack_area, 
		K_THREAD_STACK_SIZEOF(lsm6ds33_stack_area),
		lsm6ds33, NULL, NULL, NULL, 
		PRIORITY, 0, K_MSEC(60));
	k_thread_name_set(&lsm6ds33_thread_data, "LSM Thread");
#endif

#ifdef CONSUMER
	k_thread_create(&consumer_thread_data, consumer_stack_area,
			K_THREAD_STACK_SIZEOF(consumer_stack_area),
			consumer_thread, NULL, NULL, NULL,
			PRIORITY-3, 0, K_MSEC(80));
	k_thread_name_set(&consumer_thread_data, "Consumer_Thread");
#endif 

	// k_thread_create(&temperature_checker_thread_data, temperature_checker_stack_area,
	// 	K_THREAD_STACK_SIZEOF(temperature_checker_stack_area),
	// 	temperature_checker_thread, NULL, NULL, NULL,
	// 	PRIORITY, 0, K_MSEC(100));
		// k_thread_name_set(&temperature_checker_thread_data, "Temperature_Checker_Thread");
	k_timer_start(&producer_timer, K_SECONDS(normal_producer_timer_period), K_SECONDS(normal_producer_timer_period));
#ifdef CONSUMER
	k_timer_start(&consumer_timer, K_SECONDS(normal_consumer_timer_period), K_SECONDS(normal_consumer_timer_period));
#endif


}
