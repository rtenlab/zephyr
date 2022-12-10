/* main.c - Hello World demo */

/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <kernel.h>
#include <sys/printk.h>
#include "uart_i2c.h"
#include "battery/battery.h"

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

K_THREAD_STACK_DEFINE(threadA_stack_area, STACKSIZE);
static struct k_thread threadA_data;

/* Timer Declaration start.*/


void my_work_handler(struct k_work* work){
	k_wakeup(&threadA_data);
}

K_WORK_DEFINE(my_work, my_work_handler);


void my_timer_handler(struct k_timer* timer){
	k_work_submit(&my_work);
}

K_TIMER_DEFINE(my_timer, my_timer_handler, NULL);

/* Timer Declaration End.*/


enum tempMode{
	// Period remains 20 mins as the temp. is fluctuating.
	TEMP_FLUCTUATING_MODE,
	// Period equal to 1 mins for the next 60 minutes.
	TEMP_BURST_MODE,
	// Period will be equal to 60 minutes, if the temp is stable.
	TEMP_NORMAL_MODE,
};

enum batteryMode{
	// Battery is above 4 V and the values should not be changed.
	BAT_NORMAL_MODE,
	// Voltage is between (3.7-3.9]. Change the period.
	BAT_LOW_MODE1,
	// Voltage is between (3.5-3.7]. Change the period.
	BAT_LOW_MODE2,
	// Voltage is below 3.4 volts. Change to the maximum period.
	BAT_CRITICAL_MODE,

};


enum timerPeriod{
	TIMER_BURST_PERIOD=1,
	// Period 20 mins.
	BAT_PERIOD_20M=20,
	// Period 30 mins.
	BAT_PERIOD_30M=30,
	// Period 1 hour.
	BAT_PERIOD_1H=60,
	// Period 2 hours.
	BAT_PERIOD_2H=120,
	// Period 3 hours.
	BAT_PERIOD_3H=180,
	BAT_PERIOD_4H = 240,
};



uint8_t check_battery_level(void){
	float batt = battery_sample();
	if(batt >4) return BAT_NORMAL_MODE;
	else if((batt>3.70) && (batt<=4.00)) return BAT_LOW_MODE1;
	else if((batt>3.50) && (batt<=3.70)) return BAT_LOW_MODE2;
	else if(batt<=3.50) return BAT_CRITICAL_MODE;

	return BAT_NORMAL_MODE;
}
/* threadA is a static thread that is spawned automatically */

volatile uint8_t battery_mode;
volatile uint8_t period;


/**
 * @brief  Sets the battery mode variable and also set the producer consumer timer periods depending on the battery level.
 * @note   
 * @retval Curret Battery Mode returned from check_battery_level function. 
 */
int set_battery_mode(void){
	uint8_t new_batt_mode = check_battery_level();
		if(new_batt_mode == BAT_NORMAL_MODE){
			// make the period to the normal values. Prod. -> 20 and Cons. ->40.
			// Wake up all the tasks.
			if(new_batt_mode!=battery_mode)
				k_timer_start(&my_timer,K_MINUTES(BAT_PERIOD_20M), K_MINUTES(BAT_PERIOD_20M));
			
			battery_mode=new_batt_mode;
			return battery_mode;
		}
		if(new_batt_mode == BAT_LOW_MODE1){
			// change the producer period to be equal to 30 mintues and consumer 60 mins.
			// Wakeup all the tasks.
			if(new_batt_mode!=battery_mode)
				k_timer_start(&my_timer,K_MINUTES(BAT_PERIOD_30M), K_MINUTES(BAT_PERIOD_30M));
			battery_mode = new_batt_mode;
			return battery_mode;
		}
		else if(new_batt_mode == BAT_LOW_MODE2){
			// Producer period = 60 mins and consumer = 120 mins.		
			// Do not wake up the LSM and SCD task.
			if(new_batt_mode!=battery_mode)
				k_timer_start(&my_timer,K_MINUTES(BAT_PERIOD_1H), K_MINUTES(BAT_PERIOD_1H));
			battery_mode = new_batt_mode;
			return battery_mode;
		}
		else if(new_batt_mode == BAT_CRITICAL_MODE){
			// Producer period = 240 and consumer 240 mins.
			// Only wake up the SHT31 task.
			if(new_batt_mode!=battery_mode)
				k_timer_start(&my_timer,K_MINUTES(BAT_PERIOD_2H), K_MINUTES(BAT_PERIOD_2H));
			battery_mode = new_batt_mode;
			return battery_mode;
		}
	return -1;
}

void scheduler(void *dummy1, void *dummy2, void *dummy3)
{
	ARG_UNUSED(dummy1);
	ARG_UNUSED(dummy2);
	ARG_UNUSED(dummy3);
	uint8_t consumer_wakeup=0;
	while(1){
		/**
		 * Code Design for the scheduler. 
		 * 1. Battery level checking and changing the timer. 
		 * 2. Depending on the battery level wakeup minimal tasks. 
		 *
		 **/
		int ret = set_battery_mode();
		
		consumer_wakeup^=1;
		if(consumer_wakeup){
			// Wake up the consumer thread.
			;
		}
		k_sleep(K_FOREVER);
	}

}

static inline void timer_period_init(){
	period.producer_period = 20;
	period.consumer_period = 40;
}

void main(void)
{
	enable_uart_console();

	// Enable battery measurement.
	int rc = battery_measure_enable(true);
	if (rc != 0) {
		printk("Failed initialize battery measurement: %d\n", rc);
	}
	
	timer_period_init();

	k_thread_create(&threadA_data, threadA_stack_area,
			K_THREAD_STACK_SIZEOF(threadA_stack_area),
			scheduler, NULL, NULL, NULL,
			PRIORITY, 0, K_FOREVER);
	k_thread_name_set(&threadA_data, "thread_a");

	k_timer_start(&my_timer, K_MINUTES(period.producer_period), K_MINUTES(period.producer_period));
	k_thread_start(&threadA_data);
	k_thread_join(&threadA_data,K_FOREVER);
	return;
}
