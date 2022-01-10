/* main.c - Hello World demo */

/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

// #include "uart_i2c.c"
#include <zephyr.h>
#include <sys/printk.h>

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

struct k_mutex mymutex;

void scd41(void *dummy1, void *dummy2, void* dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	struct k_thread *current_thread;
	current_thread = k_current_get();
	scd41_t threadD_scd41;
	
	while(1){
		k_mutex_lock(&mymutex,K_FOREVER);
		printk("Hello from %s\n",k_thread_name_get(current_thread));
		measure_single_shot(&threadD_scd41);
		print_data_scd(&threadD_scd41);
		k_msleep(SLEEPTIME);
		k_mutex_unlock(&mymutex);
	}

}

void apds9960(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	struct k_thread *current_thread;
	apds9960_t threadC_apds9960;
	current_thread = k_current_get();
	while(1){
		// Write something to start another sensor.
		k_mutex_lock(&mymutex, K_FOREVER);
		printk("Hello from %s\n", k_thread_name_get(current_thread));
		read_proximity_data(&threadC_apds9960);
		read_als_data(&threadC_apds9960);
		print_data_apds(&threadC_apds9960);
		k_msleep(SLEEPTIME);
		k_mutex_unlock(&mymutex);
	}
}


/* threadB is a dynamic thread that is spawned by threadA */

void lsm6ds33(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	struct k_thread *current_thread;
	current_thread = k_current_get();
	while(1){
	/* invoke routine to ping-pong hello messages with threadA */
	lsm6ds33_t threadB_lsm6ds33;
	k_mutex_lock(&mymutex, K_FOREVER);
	printk("Hello from %s\n", k_thread_name_get(current_thread));
	k_msleep(SLEEPTIME);
	read_burst_data(&threadB_lsm6ds33);
	print_data(&threadB_lsm6ds33);
	k_mutex_unlock(&mymutex);
	}
}

/* threadA is a static thread that is spawned automatically */

void sht31(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	/* invoke routine to ping-pong hello messages with threadB */
	while(1){
	sht31_t threadA_sht31;
	struct k_thread *current_thread;
	current_thread = k_current_get();
	k_mutex_lock(&mymutex, K_FOREVER);
	printk("Hello from %s\n", k_thread_name_get(current_thread));
	k_msleep(SLEEPTIME);
	read_temp_hum(&threadA_sht31);
	print_data_sht(&threadA_sht31);
	k_mutex_unlock(&mymutex);
	}
}


K_THREAD_STACK_DEFINE(sht31_stack_area, STACKSIZE);
static struct k_thread sht31_data;

K_THREAD_STACK_DEFINE(lsm6ds33_stack_area, STACKSIZE);
static struct k_thread lsm6ds33_data;

K_THREAD_STACK_DEFINE(apds9960_stack_area, STACKSIZE);
static struct k_thread apds9960_data;

K_THREAD_STACK_DEFINE(scd41_stack_area, STACKSIZE);
static struct k_thread scd41_data;


void main(void)
{  
	enable_uart_console();
	configure_device();
	lsm6ds33_init();
	enable_apds_sensor();
	k_mutex_init(&mymutex);
	printk("Number of CPUS: %d\n", CONFIG_MP_NUM_CPUS);
	k_thread_create(&sht31_data, sht31_stack_area,
			K_THREAD_STACK_SIZEOF(sht31_stack_area),
			sht31, NULL, NULL, NULL,
			PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&sht31_data, "thread_A");

	k_thread_create(&lsm6ds33_data, lsm6ds33_stack_area,
			K_THREAD_STACK_SIZEOF(lsm6ds33_stack_area),
			lsm6ds33, NULL, NULL, NULL,
			PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&lsm6ds33_data, "thread_B");

	k_thread_create(&apds9960_data, apds9960_stack_area, 
			K_THREAD_STACK_SIZEOF(apds9960_stack_area),
			apds9960, NULL, NULL, NULL, 
			PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&apds9960_data, "thread_C");

	k_thread_create(&scd41_data, scd41_stack_area, 
			K_THREAD_STACK_SIZEOF(scd41_stack_area), 
			scd41, NULL, NULL, NULL,
			PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&scd41_data, "thread_D");


	k_thread_start(&sht31_data);
	k_thread_start(&lsm6ds33_data);
	k_thread_start(&apds9960_data);
	// k_thread_start(&scd41_data);
}
