#ifndef __SETUP_DEFINES_H_
#define __SETUP_DEFINES_H_
#include "sht31.c"
#include "uart_i2c.c"
#include "lsm6ds33.c"
#include "apds9960.c"
#include "scd41.c"
#include "bmp280.h"

// THREAD DEFINES
#define SHT31
#define UART
// #define LSM6DS33
#define APDS9960
#define SCD41
#define BMP280
#define BLE
#define CONSUMER
/* size of stack area used by each thread */
#define STACKSIZE 1024

#define SHT31_POS 0
#define SCD41_POS 1
#define BMP280_POS 2
#define DS18B20_POS 3

#define SWARMING_TEMP_THRESHOLD 20



#ifdef SHT31
K_THREAD_STACK_DEFINE(sht31_stack_area, STACKSIZE);
static struct k_thread sht31_thread_data;
#endif
#ifdef APDS9960
K_THREAD_STACK_DEFINE(apds9960_stack_area, STACKSIZE);
static struct k_thread apds9960_thread_data;
#endif

#ifdef BMP280
K_THREAD_STACK_DEFINE(bmp280_stack_area, STACKSIZE);
static struct k_thread bmp280_thread_data;
#endif

#ifdef LSM6DS33
K_THREAD_STACK_DEFINE(lsm6ds33_stack_area, STACKSIZE);
static struct k_thread lsm6ds33_thread_data;
#endif

#ifdef CONSUMER
K_THREAD_STACK_DEFINE(consumer_stack_area, STACKSIZE);
static struct k_thread consumer_thread_data;
#endif

K_THREAD_STACK_DEFINE(temperature_checker_stack_area, STACKSIZE);
static struct k_thread temperature_checker_thread_data;

// THREAD DEFINES END


// Let's first define message queue.
#define MSGQ_LENGTH 8
#define MSGQ_BYTE_BOUNDARY 4

// Enumeration to use inside the new struct ble_data_t. This enum will give names\
	to different type of sensor data	
enum data_type{
	SENSOR_SHT31,
	SENSOR_APDS9960,
	SENSOR_BMP280,
	SENSOR_LSM6DS33,
	SENSOR_SCD41,
	SENSOR_DS18B20,
};

/**
 * @brief  Struct to define the data structure used for data passing between threads.
 */
typedef struct {
	enum data_type type;
	float period;
	union 
	{
		// scd41_t scd41_data;
		sht31_t sht31_data;
		lsm6ds33_t lsm6ds33_data;
		bmp280_t bmp280_data;
		apds9960_t apds_cls_data;
		// int16_t ds18b20_temp_data;
	};
}ble_data_t;

/* Create a MSG Queue for the threads to use for data passing*/
K_MSGQ_DEFINE(my_msgq, sizeof(ble_data_t), MSGQ_LENGTH, MSGQ_BYTE_BOUNDARY);


// END OF MESSAGE QUEUE DEFINES



/***WORKQ defines:***/
/**
 * @brief  Work Handler function. This will be ran by a thread in backend depending on the priority.
 * @note   
 * @param  struct k_work*: What wok needs to be done.
 * @retval None
 */
void producer_work_handler(struct k_work *work){
	printk("Timer fired!!!\n");
#ifdef SHT31
	k_wakeup(&sht31_thread_data);
#endif
#ifdef APDS9960
	k_wakeup(&apds9960_thread_data);
#endif
#ifdef BMP280
	k_wakeup(&bmp280_thread_data);
#endif
#ifdef LSM6DS33
	k_wakeup(&lsm6ds33_thread_data);
#endif
}

void consumer_work_handler(struct k_work * work){
	printk("Consumer timer Fired!!!\n");
	k_wakeup(&consumer_thread_data);
}
// Define a work and it's work handler
K_WORK_DEFINE(producer_work, producer_work_handler);
K_WORK_DEFINE(consumer_work, consumer_work_handler);

/** WORKQ DEFINES END**/

/*** TIMER DEFINES ***/
// Define a function that will be called when a timer expires.
void producer_timer_handler(struct k_timer* dummy){
	k_work_submit(&producer_work);
}

void consumer_timer_handler(struct k_timer* dummy){
	k_work_submit(&consumer_work);
}

// Define a timer.
K_TIMER_DEFINE(producer_timer, producer_timer_handler, NULL);
K_TIMER_DEFINE(consumer_timer, consumer_timer_handler, NULL);

/*** TIMER DEFINES END ***/

#endif