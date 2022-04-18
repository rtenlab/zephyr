/* main.c - Hello World demo */

/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #include "uart_i2c.c"
#include <zephyr.h>
#include <sys/printk.h>
#include <errno.h>

#include "sht31.c"
#include "uart_i2c.c"
#include "lsm6ds33.c"
#include "apds9960.c"
#include "scd41.c"



/*
 * The hello world demo has two threads that utilize semaphores and sleeping
 * to take turns printing a greeting message at a controlled rate. The demo
 * shows both the static and dynamic approaches for spawning a thread; a real
 * world application would likely use the static approach for both threads.
 */

#define PIN_THREADS (IS_ENABLED(CONFIG_SMP)		  \
		     && IS_ENABLED(CONFIG_SCHED_CPU_MASK) \
		     && (CONFIG_MP_NUM_CPUS > 1))

/* size of stack area used by each thread */
#define STACKSIZE 1024

/* scheduling priority used by each thread */
#define PRIORITY 7

/* delay between greetings (in ms) */
#define SLEEPTIME 500


/*
 * @param my_name      thread identification string
 * @param my_sem       thread's own semaphore
 * @param other_sem    other thread's semaphore
 */
/* define semaphores */

// K_SEM_DEFINE(threadA_sem, 1, 1);	/* starts off "available" */
// K_SEM_DEFINE(threadB_sem, 0, 1);	/* starts off "not available" */
enum data_type{
	SENSOR_SHT31,
	SENSOR_APDS9960,
	SENSOR_BMP280,
	SENSOR_LSM6DS33,
	SENSOR_SCD41,
	SENSOR_DS18B20,
};

typedef struct {
	// id->0: sht data; id->1: apds data
	enum data_type type;
	union 
	{
		// scd41_t scd41_data;
		sht31_t sht31_data;
		// lsm6ds33_t lsm6ds33_data;
		// bmp280_t bmp280_data;
		apds9960_t apds_cls_data;
		// int16_t ds18b20_temp_data;
	};
}ble_data_t;


struct k_mutex mymutex;

/* Create a MSG Queue for the threads to use for data passing*/
// struct k_msgq my_msgq;
K_MSGQ_DEFINE(my_msgq, sizeof(ble_data_t), 3, 4);

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
	while(1){
		k_mutex_lock(&mymutex, K_FOREVER);
		printk("Hello from %s\n", k_thread_name_get(current_thread));
		// read_proximity_data(&threadC_apds9960);
		read_als_data(&apds_local_data.apds_cls_data);
		k_msgq_put(&my_msgq, &apds_local_data, K_FOREVER);
		// print_data_apds(&shared_buffer[1].apds_cls_data);
		k_msleep(SLEEPTIME);
		k_mutex_unlock(&mymutex);
		k_sleep(K_FOREVER);
	}
	
	return;
}




/* threadA is a static thread that is spawned automatically */

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
	while(1){
		k_mutex_lock(&mymutex, K_FOREVER);
		printk("Hello from %s\n", k_thread_name_get(current_thread));
		k_msleep(SLEEPTIME);
		read_temp_hum(&sht31_local_data.sht31_data);
		k_msgq_put(&my_msgq, &sht31_local_data, K_FOREVER);
		// print_data_sht(&shared_buffer[1].sht31_data);
		k_mutex_unlock(&mymutex);
		k_sleep(K_FOREVER);
	}
	return;
}

char* enum_to_string(ble_data_t* data){
	if(data->type == SENSOR_APDS9960){
		return "APDS9960";
	}
	else if(data->type == SENSOR_SHT31){
		return "SHT31";
	}
	return "NOT VALID";
}

void consumer_thread(void* dummy1, void* dummy2, void* dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	ble_data_t consumer_local_data;
	while(1){
		int8_t ret = k_msgq_get(&my_msgq, &consumer_local_data, K_MSEC(10));
		if(ret!=0){
			printk("Couldn't find data\n");
		}
		printk("Data type: %s\t Data value: (Nothing for now)\n", enum_to_string(&consumer_local_data));
		k_sleep(K_FOREVER);
	}

}

K_THREAD_STACK_DEFINE(sht31_stack_area, STACKSIZE);
static struct k_thread sht31_thread_data;

K_THREAD_STACK_DEFINE(apds9960_stack_area, STACKSIZE);
static struct k_thread apds9960_thread_data;

K_THREAD_STACK_DEFINE(consumer_stack_area, STACKSIZE);
static struct k_thread consumer_thread_data;

/**
 * @brief  Work Handler function. This will be ran by a thread in backend depending on the priority.
 * @note   
 * @param  struct k_work*: What wok needs to be done.
 * @retval None
 */
void my_work_handler(struct k_work *work){
	k_thread_resume(&sht31_thread_data);
	// k_thread_start(&lsm6ds33_data);
	k_thread_resume(&apds9960_thread_data);
	k_thread_resume(&consumer_thread_data);
}

// Define a work and it's work handler
K_WORK_DEFINE(my_work, my_work_handler);

// Define a function that will be called when a timer expires.
void my_timer_handler(struct k_timer* dummy){
	k_work_submit(&my_work);
}
// Define a timer.
K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);


volatile uint8_t count;
void main(void)
{  
	enable_uart_console();
	configure_device();
	lsm6ds33_init();
	enable_apds_sensor();
	k_mutex_init(&mymutex);
	printk("Number of CPUS: %d\n", CONFIG_MP_NUM_CPUS);

	k_thread_create(&sht31_thread_data, sht31_stack_area,
			K_THREAD_STACK_SIZEOF(sht31_stack_area),
			sht31, NULL, NULL, NULL,
			PRIORITY, 0, K_MSEC(100));
	k_thread_name_set(&sht31_thread_data, "SHT_Thread");



	k_thread_create(&apds9960_thread_data, apds9960_stack_area, 
			K_THREAD_STACK_SIZEOF(apds9960_stack_area),
			apds9960, NULL, NULL, NULL, 
			PRIORITY, 0, K_MSEC(100));
	k_thread_name_set(&apds9960_thread_data, "APDS Thread");

	k_timer_start(&my_timer, K_SECONDS(2), K_SECONDS(4));


	k_thread_create(&consumer_thread_data, consumer_stack_area,
			K_THREAD_STACK_SIZEOF(consumer_stack_area),
			consumer_thread, NULL, NULL, NULL,
			PRIORITY-3, 0, K_MSEC(100));
	k_thread_name_set(&consumer_thread_data, "Consumer_Thread");

}
