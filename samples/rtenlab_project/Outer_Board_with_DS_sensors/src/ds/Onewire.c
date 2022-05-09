#include "Onewire.h"

unsigned char ROM_NO[8];
uint8_t LastDiscrepancy;
uint8_t LastFamilyDiscrepancy;
bool LastDeviceFlag;

const struct device *dev_ds18b20;

static inline void udelay(int secs){
	k_busy_wait(secs);
}


uint8_t reset(void){
	// int key = irq_lock();
	uint8_t r;
	gpio_pin_configure(dev_ds18b20, DS_SENSOR_PIN, GPIO_OUTPUT);
	gpio_pin_set(dev_ds18b20,DS_SENSOR_PIN,LOW);
	udelay(480);
	gpio_pin_set(dev_ds18b20,DS_SENSOR_PIN,HIGH);
	udelay(70);
	// while((gpio_pin_get(dev_ds18b20,DS_SENSOR_PIN))!=0){
	// 	r = gpio_pin_get(dev_ds18b20,DS_SENSOR_PIN);
	// 	printk("stuck in teh while loopwith value of r: %d\n",r);
	// }
	gpio_pin_configure(dev_ds18b20, DS_SENSOR_PIN, GPIO_INPUT);
	r = gpio_pin_get(dev_ds18b20,DS_SENSOR_PIN);
	udelay(410);
	// irq_unlock(key);
	return !r;
}

uint8_t read_bit(void){
	uint8_t r;
	int key = irq_lock();
	gpio_pin_configure(dev_ds18b20, DS_SENSOR_PIN, GPIO_OUTPUT);
	gpio_pin_set(dev_ds18b20,DS_SENSOR_PIN,LOW);
	udelay(3);
	// gpio_pin_set(dev_ds18b20,DS_SENSOR_PIN,HIGH);
	gpio_pin_configure(dev_ds18b20, DS_SENSOR_PIN, GPIO_INPUT);
	udelay(10);	
	r = gpio_pin_get(dev_ds18b20,DS_SENSOR_PIN);
	irq_unlock(key);
	udelay(53);
	return r;
}

uint8_t read(void){
	uint8_t bitMask;
	uint8_t r = 0;
	for(bitMask=0x01; bitMask;bitMask<<=1){
		if(read_bit()){
			r|=bitMask;
			// udelay(1);
		}
	}
	return r;
}

void write_bit(uint8_t v){
	// printk("Outputting bit: %d\n",v);
	if(v&1){
		int key = irq_lock();
		gpio_pin_set(dev_ds18b20,DS_SENSOR_PIN,LOW);
		gpio_pin_configure(dev_ds18b20, DS_SENSOR_PIN, GPIO_OUTPUT);
		udelay(10);
		gpio_pin_set(dev_ds18b20,DS_SENSOR_PIN,HIGH);
		irq_unlock(key);
		udelay(55);
	}
	else{
		int key = irq_lock();
		gpio_pin_set(dev_ds18b20,DS_SENSOR_PIN,LOW);
		gpio_pin_configure(dev_ds18b20, DS_SENSOR_PIN, GPIO_OUTPUT);
		udelay(65);
		gpio_pin_set(dev_ds18b20,DS_SENSOR_PIN,HIGH);
		irq_unlock(key);
		udelay(5);
	}
	return;	
}

void write(uint8_t v){
	uint8_t bitMask;
	for(bitMask = 0x01; bitMask; bitMask<<=1){
		write_bit((bitMask&v)?1:0);
		// udelay(1);
	}
	return;
}

bool search(uint8_t *newAddr)
{
   bool search_mode = true;
   uint8_t id_bit_number;
   uint8_t last_zero, rom_byte_number;
   bool    search_result;
   uint8_t id_bit, cmp_id_bit;

   unsigned char rom_byte_mask, search_direction;

   // initialize for search
   id_bit_number = 1;
   last_zero = 0;
   rom_byte_number = 0;
   rom_byte_mask = 1;
   search_result = false;

   // if the last call was not the last one
   if (!LastDeviceFlag) {
      // 1-Wire reset
      if (!reset()) {
         // reset the search
         LastDiscrepancy = 0;
         LastDeviceFlag = false;
         LastFamilyDiscrepancy = 0;
         return false;
      }

      // issue the search command
      if (search_mode == true) {
        write(0xF0);   // NORMAL SEARCH
      } else {
        write(0xEC);   // CONDITIONAL SEARCH
      }

      // loop to do the search
      do
      {
         // read a bit and its complement
         id_bit = read_bit();
         cmp_id_bit = read_bit();

         // check for no devices on 1-wire
         if ((id_bit == 1) && (cmp_id_bit == 1)) {
            break;
         } else {
            // all devices coupled have 0 or 1
            if (id_bit != cmp_id_bit) {
               search_direction = id_bit;  // bit write value for search
            } else {
               // if this discrepancy if before the Last Discrepancy
               // on a previous next then pick the same as last time
               if (id_bit_number < LastDiscrepancy) {
                  search_direction = ((ROM_NO[rom_byte_number] & rom_byte_mask) > 0);
               } else {
                  // if equal to last pick 1, if not then pick 0
                  search_direction = (id_bit_number == LastDiscrepancy);
               }
               // if 0 was picked then record its position in LastZero
               if (search_direction == 0) {
                  last_zero = id_bit_number;

                  // check for Last discrepancy in family
                  if (last_zero < 9)
                     LastFamilyDiscrepancy = last_zero;
               }
            }

            // set or clear the bit in the ROM byte rom_byte_number
            // with mask rom_byte_mask
            if (search_direction == 1)
              ROM_NO[rom_byte_number] |= rom_byte_mask;
            else
              ROM_NO[rom_byte_number] &= ~rom_byte_mask;

            // serial number search direction write bit
            write_bit(search_direction);

            // increment the byte counter id_bit_number
            // and shift the mask rom_byte_mask
            id_bit_number++;
            rom_byte_mask <<= 1;

            // if the mask is 0 then go to new SerialNum byte rom_byte_number and reset mask
            if (rom_byte_mask == 0) {
                rom_byte_number++;
                rom_byte_mask = 1;
            }
         }
      }
      while(rom_byte_number < 8);  // loop until through all ROM bytes 0-7

      // if the search was successful then
      if (!(id_bit_number < 65)) {
         // search successful so set LastDiscrepancy,LastDeviceFlag,search_result
         LastDiscrepancy = last_zero;

         // check for last device
         if (LastDiscrepancy == 0) {
            LastDeviceFlag = true;
         }
         search_result = true;
      }
   }

   // if no device found then reset counters so next 'search' will be like a first
   if (!search_result || !ROM_NO[0]) {
      LastDiscrepancy = 0;
      LastDeviceFlag = false;
      LastFamilyDiscrepancy = 0;
      search_result = false;
   } else {
      for (int i = 0; i < 8; i++) newAddr[i] = ROM_NO[i];
   }
   return search_result;
}

void select(const uint8_t rom[8])
{
    uint8_t i;

    write(0x55);           // Choose ROM

    for (i = 0; i < 8; i++) write(rom[i]);
}

void reset_search(void)
{
  // reset the search state
  LastDiscrepancy = 0;
  LastDeviceFlag = false;
  LastFamilyDiscrepancy = 0;
  for(int i = 7; ; i--) {
    ROM_NO[i] = 0;
    if ( i == 0) break;
  }
}
void skip()
{
    write(0xCC);           // Skip ROM
}

