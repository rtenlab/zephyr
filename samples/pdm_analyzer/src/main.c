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

// For the Speakers you see
#include <audio/dmic.h>

// For the FFT
#include <stdlib.h>
#include <arm_math.h>
#include <arm_const_structs.h>

#define AUDIO_FREQ		16000
#define CHAN_SIZE		16
#define PCM_BLK_SIZE_MS		((AUDIO_FREQ/1000) * sizeof(int16_t))

//#define USE_NRFX_PDM
#define USE_ARDUINO_PDM

#if defined(USE_NRFX_PDM)

#define NUM_MS		400
#define MARGIN_MS 100

// slab memory allocation pool for dmic_nrfx_pdm driver
K_MEM_SLAB_DEFINE(rx_mem_slab, PCM_BLK_SIZE_MS, NUM_MS + MARGIN_MS, 1); 

// buffer for application
uint8_t rx_block[NUM_MS][PCM_BLK_SIZE_MS];

#elif defined (USE_ARDUINO_PDM)

// For Arduino
short sampleBuffer[256];
volatile int samplesRead;

#endif

struct pcm_stream_cfg mic_streams = {
	.pcm_rate = AUDIO_FREQ,
	.pcm_width = CHAN_SIZE,
	.block_size = PCM_BLK_SIZE_MS,
#if defined(USE_NRFX_PDM)
	.mem_slab = &rx_mem_slab,
#endif
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

// This is the buffer that holds the samples
#define FFT_SAMPLES 400
#define FFT_SAMPLES_HALF (FFT_SAMPLES/2)
static float32_t inputSignal[FFT_SAMPLES];

// This is for the FFT
static float32_t complexFFT[FFT_SAMPLES], realFFT[FFT_SAMPLES_HALF],
		imagFFT[FFT_SAMPLES_HALF], angleFFT[FFT_SAMPLES_HALF],
		powerFFT[FFT_SAMPLES_HALF];

uint32_t fftSize = FFT_SAMPLES;
uint32_t ifftFlag = 0;
arm_rfft_fast_instance_f32 S;
uint32_t maxIndex = 0;
arm_status status;
float32_t maxValue;
int i;

void main(void)
{
  const struct device *dev = device_get_binding(
			CONFIG_UART_CONSOLE_ON_DEV_NAME);

  const struct device *mic_dev = device_get_binding(DT_LABEL(DT_NODELABEL(pdm0)));
  
  uint32_t dtr = 0;
  int ret;

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

#if defined(USE_NRFX_PDM)
  for (int i = 0; i < NUM_MS; i++) {
    for (int j = 0; j < PCM_BLK_SIZE_MS; j++) 
      rx_block[i][j] = 0xff; // initialize 
  }

  while (1) {
  	ret = dmic_trigger(mic_dev, DMIC_TRIGGER_START);
  	if (ret < 0) {
  		printk("microphone start trigger error\n");
      continue;
  	}
  
    printk("signal_sampling_started\n");
  
  	/* Acquire microphone audio */
  	for (int ms = 0; ms < NUM_MS; ms++) {
      int rx_size;
  		ret = dmic_read(mic_dev, 0, (void**)&rx_block[ms], &rx_size, 2000);
  		if (ret < 0) {
  			printk("microphone audio read error\n");
  	    dmic_trigger(mic_dev, DMIC_TRIGGER_STOP);
        k_sleep(K_MSEC(300));
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
  
    short minwave = 30000;
    short maxwave = -30000;

  	for (int i = 0; i < NUM_MS; i++) {
      short *pcm_out = (short*)rx_block[i];

      for (int j = 0; j < PCM_BLK_SIZE_MS / 2; j++) {
        //printk("0x%04x,\n", pcm_out[j]);
        printk("%d ", pcm_out[j]);
        minwave = min(minwave, pcm_out[j]);
        maxwave = max(maxwave, pcm_out[j]);
      }
      printk("\nmic: %d\n", maxwave - minwave);
    }
  }

#elif defined(USE_ARDUINO_PDM)
  // Arduino approach: polling
  struct k_timer my_timer;
  
  while (1) {
    samplesRead = 0;

    int32_t samples = 400;
    
    short minwave = 30000;
    short maxwave = -30000;

    while (samples > 0) {
      if (!samplesRead) {
        k_yield();
        continue;
      }
      for (int i = 0; i < samplesRead; i++) {
        minwave = min(sampleBuffer[i], minwave);
        maxwave = max(sampleBuffer[i], maxwave);
        samples--;
        //write the values into the file here
        //printk("%d ", sampleBuffer[i]); 

        inputSignal[399 - samples] = (float32_t)sampleBuffer[i];

        //if ((i % 16) == 0) printk("\n"); This is the original

        if ((i % 16) == 0) {
          // printk("\n");
          ret = dmic_trigger(mic_dev, DMIC_TRIGGER_STOP);
          if (ret < 0) {
            printk("microphone stop trigger error\n");
            continue;
            }
          }
      }
      // clear the read count
      samplesRead = 0;
      // printk("mic: %d\n", maxwave - minwave);
    }
    //printk("\nmic: %d\n", maxwave - minwave);

    // Include the FFT here
    printk("Begin FFT Ops\n");
    status = ARM_MATH_SUCCESS;
    status = arm_rfft_fast_init_f32(&S, fftSize);
    printk("The FFT has been initialized\n");
    arm_rfft_fast_f32(&S, inputSignal, complexFFT, ifftFlag);
    printk("FFT has been completed\n");
    float32_t DCoffset = complexFFT[0];

    for (i = 0; i < (FFT_SAMPLES / 2) - 1; i++) {
      realFFT[i] = complexFFT[i * 2];
      imagFFT[i] = complexFFT[(i * 2) + 1];
    }

    printk("The complex numbers have been made\n");
    
    for (i = 0; i < FFT_SAMPLES / 2; i++) {
      angleFFT[i] = atan2f(imagFFT[i], realFFT[i]);
    }

    printk("Phase Responses has been constructed\n");

    arm_cmplx_mag_squared_f32(complexFFT, powerFFT, FFT_SAMPLES_HALF);

    printk("Power Spectrum has been contructed\n");

    arm_max_f32(&powerFFT[1], FFT_SAMPLES_HALF - 1, &maxValue, &maxIndex);
    // correct index

    printk("Real Test: %f, Imag Test: %f\n", complexFFT[3], complexFFT[4]);
    printk("Power Test: %f\n", powerFFT[4]);
    
    maxIndex += 1;

    k_sleep(K_MSEC(300));
  }
#endif
}
