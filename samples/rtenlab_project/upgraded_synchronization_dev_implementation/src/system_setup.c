#include "system_setup.h"
#include "sht31.h"
#include "apds9960.h"

void sht31(void* dummy1, void* dummy2, void* dummy3){
    ARG_UNUSED(dummy1);
    ARG_UNUSED(dummy2);
    ARG_UNUSED(dummy3);
    sht31_t curr, prev;
    float delta=0.0;
    uint32_t curr_timestamp, prev_timestamp;    
    while(1){
        curr_timestamp = k_uptime_get_32();
        read_temp_hum(&curr);
        prev_timestamp = curr_timestamp;
        printk("Hello from the SHT31 thread.\n");
        delta = (curr.temp - prev.temp);
        memcpy(&prev, &curr, sizeof(curr));
        k_sleep(K_FOREVER);
    }

}


void apds9960(void* dummy1, void* dummy2, void* dummy3){
    ARG_UNUSED(dummy1);
    ARG_UNUSED(dummy2);
    ARG_UNUSED(dummy3);
    apds9960_t curr, prev;

    uint32_t curr_timestamp, prev_timestamp;    
    while(1){
        curr_timestamp = k_uptime_get_32();
        read_proximity_data(&curr);
        read_als_data(&curr);
        prev_timestamp = curr_timestamp;
        printk("Hello from the APDS thread.\n");
        k_sleep(K_FOREVER);
    }
}