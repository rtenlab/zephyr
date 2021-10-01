/*
 * Copyright (c) 2016 Intel Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <sys/printk.h>
#include <sys/util.h>
#include <string.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>

#include <audio/dmic.h>

#define AUDIO_FREQ		16000
#define CHAN_SIZE		16
#define PCM_BLK_SIZE_MS		((AUDIO_FREQ/1000) * sizeof(int16_t))

#define NUM_MS		1000

K_MEM_SLAB_DEFINE(rx_mem_slab, PCM_BLK_SIZE_MS, NUM_MS, 1);

struct pcm_stream_cfg mic_streams = {
	.pcm_rate = AUDIO_FREQ,
	.pcm_width = CHAN_SIZE,
	.block_size = PCM_BLK_SIZE_MS,
	.mem_slab = &rx_mem_slab,
};

struct dmic_cfg cfg = {
	.io = {
		/* requesting a pdm freq aroud 1280KHz */
		.min_pdm_clk_freq = 1200000,
		.max_pdm_clk_freq = 1300000,
	},
	.streams = &mic_streams,
	.channel = {
		.req_num_chan = 1,
    .req_num_streams = 1,
	},
};

void *rx_block[NUM_MS];
size_t rx_size = PCM_BLK_SIZE_MS;

static inline short min(short a, short b)
{
  if (a > b) return b;
  return a;
}
static inline short max(short a, short b)
{
  if (a < b) return b;
  return a;
}

void main(void)
{
	const struct device *dev = device_get_binding(
			CONFIG_UART_CONSOLE_ON_DEV_NAME);

  const struct device *mic_dev = device_get_binding(DT_LABEL(DT_NODELABEL(pdm0)));

	uint32_t dtr = 0;
  int ret, i;
  uint32_t ms;

	if (usb_enable(NULL)) {
		return;
	}

	/* Poll if the DTR flag was set, optional */
	while (!dtr) {
		uart_line_ctrl_get(dev, UART_LINE_CTRL_DTR, &dtr);
	}

	if (strlen(CONFIG_UART_CONSOLE_ON_DEV_NAME) !=
	    strlen("CDC_ACM_0") ||
	    strncmp(CONFIG_UART_CONSOLE_ON_DEV_NAME, "CDC_ACM_0",
		    strlen(CONFIG_UART_CONSOLE_ON_DEV_NAME))) {
		printk("Error: Console device name is not USB ACM\n");

		return;
	}
  
  k_sleep(K_SECONDS(5));
  printk("Hello World! %s\n", CONFIG_ARCH);
  printk("Mic config start\n");

	if (!mic_dev) {
		printk("Could not get pointer to %s device\n",
			DT_LABEL(DT_NODELABEL(pdm0)));
		return;
	}

	ret = dmic_configure(mic_dev, &cfg);
	if (ret < 0) {
		printk("microphone configuration error\n");
		return;
	}

  while (1) {
  	ret = dmic_trigger(mic_dev, DMIC_TRIGGER_START);
  	if (ret < 0) {
  		printk("microphone start trigger error\n");
      continue;
  	}
  
    printk("signal_sampling_started\n");
  
  	/* Acquire microphone audio */
  	for (ms = 0U; ms < NUM_MS; ms++) {
  		ret = dmic_read(mic_dev, 0, &rx_block[ms], &rx_size, 2000);
  		if (ret < 0) {
  			printk("microphone audio read error\n");
  	    dmic_trigger(mic_dev, DMIC_TRIGGER_STOP);
        break;
  		}
  	}
    if (ret < 0) continue;
  
  	printk("signal_sampling_stopped\n");
  
  	ret = dmic_trigger(mic_dev, DMIC_TRIGGER_STOP);
  	if (ret < 0) {
  		printk("microphone stop trigger error\n");
      continue;
  	}
  	int j;
  
    short minwave = 30000;
    short maxwave = -30000;

  	for (i = 0; i < NUM_MS; i++) {
  		uint16_t *pcm_out = rx_block[i];
  
      
  		for (j = 0; j < rx_size/2; j++) {
  			//printk("0x%04x,\n", pcm_out[j]);
  			printk("%d ", pcm_out[j]);
        minwave = min(minwave, pcm_out[j]);
        maxwave = max(maxwave, pcm_out[j]);
  		}
      printk("\nmic: %d\n", maxwave - minwave);
      
  	}
  }
}
