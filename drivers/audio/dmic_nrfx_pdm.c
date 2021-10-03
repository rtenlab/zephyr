/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <audio/dmic.h>
#include <drivers/clock_control/nrf_clock_control.h>
#include <nrfx_pdm.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(dmic_nrfx_pdm, CONFIG_AUDIO_DMIC_LOG_LEVEL);

//#define USE_NRFX_PDM
#define USE_ARDUINO_PDM

struct dmic_nrfx_pdm_drv_data {
	struct onoff_manager *clk_mgr;
	struct onoff_client clk_cli;
	struct k_mem_slab *mem_slab;
	uint32_t block_size;
	struct k_msgq rx_queue;
	bool request_clock : 1;
	bool configured    : 1;
	volatile bool active;
	volatile bool stopping;
};

struct dmic_nrfx_pdm_drv_cfg {
	nrfx_pdm_event_handler_t event_handler;
	nrfx_pdm_config_t nrfx_def_cfg;
	enum clock_source {
		PCLK32M,
		PCLK32M_HFXO,
		ACLK
	} clk_src;
};

static void free_buffer(struct dmic_nrfx_pdm_drv_data *drv_data, void *buffer)
{
	k_mem_slab_free(drv_data->mem_slab, &buffer);
	LOG_DBG("Freed buffer %p", buffer);
}

static void event_handler(const struct device *dev, const nrfx_pdm_evt_t *evt)
{
	struct dmic_nrfx_pdm_drv_data *drv_data = dev->data;
	int ret;
	bool stop = false;

	if (evt->buffer_requested) {
		void *buffer;
		nrfx_err_t err;

		ret = k_mem_slab_alloc(drv_data->mem_slab, &buffer, K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("Failed to allocate buffer: %d", ret);
			stop = true;
		} else {
			err = nrfx_pdm_buffer_set(buffer,
						  drv_data->block_size / 2);
			if (err != NRFX_SUCCESS) {
				LOG_ERR("Failed to set buffer: 0x%08x", err);
				stop = true;
			}
      //LOG_ERR("event_handler: alloc buffer %p; nrfx_pdm_buffer_set\n", buffer);
		}
	}

	if (drv_data->stopping) {
		if (evt->buffer_released) {
			free_buffer(drv_data, evt->buffer_released);
		}

		if (drv_data->active) {
			drv_data->active = false;
			if (drv_data->request_clock) {
				(void)onoff_release(drv_data->clk_mgr);
			}
		}
    //LOG_ERR("event_handler: stopping\n");
	} else if (evt->buffer_released) {
		ret = k_msgq_put(&drv_data->rx_queue,
				 &evt->buffer_released,
				 K_NO_WAIT);
		if (ret < 0) {
			LOG_ERR("No room in RX queue");
			stop = true;

			//free_buffer(drv_data, evt->buffer_released); // BUG?
		} else {
			LOG_DBG("Queued buffer %p", evt->buffer_released);
		}
	  free_buffer(drv_data, evt->buffer_released); // FIXME: this should be released here, not above
	}

	if (stop) {
		nrfx_pdm_stop();
		drv_data->stopping = true;
    //LOG_ERR("event_handler: stop\n");
	}
}

static bool is_better(uint32_t freq,
		      uint8_t ratio,
		      uint32_t req_rate,
		      uint32_t *best_diff,
		      uint32_t *best_rate,
		      uint32_t *best_freq)
{
	uint32_t act_rate = freq / ratio;
	uint32_t diff = act_rate >= req_rate ? (act_rate - req_rate)
					     : (req_rate - act_rate);

	LOG_DBG("Freq %u, ratio %u, act_rate %u", freq, ratio, act_rate);

	if (diff < *best_diff) {
		*best_diff = diff;
		*best_rate = act_rate;
		*best_freq = freq;
		return true;
	}

	return false;
}

static bool check_pdm_frequencies(const struct dmic_nrfx_pdm_drv_cfg *drv_cfg,
				  nrfx_pdm_config_t *config,
				  const struct dmic_cfg *pdm_cfg,
				  uint8_t ratio,
				  uint32_t *best_diff,
				  uint32_t *best_rate,
				  uint32_t *best_freq)
{
	uint32_t req_rate = pdm_cfg->streams[0].pcm_rate;
	bool better_found = false;

	if (IS_ENABLED(CONFIG_SOC_SERIES_NRF53X)) {
		const uint32_t src_freq =
			(NRF_PDM_HAS_MCLKCONFIG && drv_cfg->clk_src == ACLK)
			/* The DMIC_NRFX_PDM_DEVICE() macro contains build
			 * assertions that make sure that the ACLK clock
			 * source is only used when it is available and only
			 * with the "hfclkaudio-frequency" property defined,
			 * but the default value of 0 here needs to be used
			 * to prevent compilation errors when the property is
			 * not defined (this expression will be eventually
			 * optimized away then).
			 */
			? DT_PROP_OR(DT_NODELABEL(clock), hfclkaudio_frequency,
				     0)
			: 32*1000*1000UL;
		uint32_t req_freq = req_rate * ratio;
		/* As specified in the nRF5340 PS:
		 *
		 * PDMCLKCTRL = 4096 * floor(f_pdm * 1048576 /
		 *                           (f_source + f_pdm / 2))
		 * f_actual = f_source / floor(1048576 * 4096 / PDMCLKCTRL)
		 */
		uint32_t clk_factor = (uint32_t)((req_freq * 1048576ULL) /
						 (src_freq + req_freq / 2));
		uint32_t act_freq = src_freq / (1048576 / clk_factor);

		if (act_freq >= pdm_cfg->io.min_pdm_clk_freq &&
		    act_freq <= pdm_cfg->io.max_pdm_clk_freq &&
		    is_better(act_freq, ratio, req_rate,
			      best_diff, best_rate, best_freq)) {
			config->clock_freq = clk_factor * 4096;

			better_found = true;
		}
	} else { /* -> !IS_ENABLED(CONFIG_SOC_SERIES_NRF53X)) */
		static const struct {
			uint32_t       freq_val;
			nrf_pdm_freq_t freq_enum;
		} freqs[] = {
			{ 1000000, NRF_PDM_FREQ_1000K },
			{ 1032000, NRF_PDM_FREQ_1032K },
			{ 1067000, NRF_PDM_FREQ_1067K },
#if defined(PDM_PDMCLKCTRL_FREQ_1231K)
			{ 1231000, NRF_PDM_FREQ_1231K },
#endif
#if defined(PDM_PDMCLKCTRL_FREQ_1280K)
			{ 1280000, NRF_PDM_FREQ_1280K },
#endif
#if defined(PDM_PDMCLKCTRL_FREQ_1333K)
			{ 1333000, NRF_PDM_FREQ_1333K }
#endif
		};

		for (int i = 0; i < ARRAY_SIZE(freqs); ++i) {
			uint32_t freq_val = freqs[i].freq_val;

			if (freq_val < pdm_cfg->io.min_pdm_clk_freq) {
				continue;
			}
			if (freq_val > pdm_cfg->io.max_pdm_clk_freq) {
				break;
			}

			if (is_better(freq_val, ratio, req_rate,
				      best_diff, best_rate, best_freq)) {
				config->clock_freq = freqs[i].freq_enum;

				/* Stop if an exact rate match is found. */
				if (*best_diff == 0) {
					return true;
				}

				better_found = true;
			}

			/* Since frequencies are are in ascending order, stop
			 * checking next ones for the current ratio after
			 * resulting PCM rate goes above the one requested.
			 */
			if ((freq_val / ratio) > req_rate) {
				break;
			}
		}
	}

	return better_found;
}

/* Finds clock settings that give the PCM output rate closest to that requested,
 * taking into account the hardware limitations.
 */
static bool find_suitable_clock(const struct dmic_nrfx_pdm_drv_cfg *drv_cfg,
				nrfx_pdm_config_t *config,
				const struct dmic_cfg *pdm_cfg)
{
	uint32_t best_diff = UINT32_MAX;
	uint32_t best_rate;
	uint32_t best_freq;

#if NRF_PDM_HAS_RATIO_CONFIG
	static const struct {
		uint8_t         ratio_val;
		nrf_pdm_ratio_t ratio_enum;
	} ratios[] = {
		{  64, NRF_PDM_RATIO_64X },
		{  80, NRF_PDM_RATIO_80X }
	};

	for (int r = 0; best_diff != 0 && r < ARRAY_SIZE(ratios); ++r) {
		uint8_t ratio = ratios[r].ratio_val;

		if (check_pdm_frequencies(drv_cfg, config, pdm_cfg, ratio,
					  &best_diff, &best_rate, &best_freq)) {
			config->ratio = ratios[r].ratio_enum;

			/* Look no further if a configuration giving the exact
			 * PCM rate is found.
			 */
			if (best_diff == 0) {
				break;
			}
		}
	}
#else
	uint8_t ratio = 64;

	(void)check_pdm_frequencies(drv_cfg, config, pdm_cfg, ratio,
				    &best_diff, &best_rate, &best_freq);
#endif

	if (best_diff == UINT32_MAX) {
		return false;
	}

	LOG_INF("PDM clock frequency: %u, actual PCM rate: %u",
		best_freq, best_rate);
	return true;
}

#if defined(USE_ARDUINO_PDM)
struct PDMDoubleBuffer 
{
  uint8_t* _buffer[2];
  int _size;
  volatile int _length[2];
  volatile int _readOffset[2];
  volatile int _index;
};

void doubleBuffer_reset(struct PDMDoubleBuffer *p, uint8_t *buf1, uint8_t *buf2, int size)
{
  p->_size = size;
  p->_buffer[0] = buf1;
  p->_buffer[1] = buf2;

  memset(p->_buffer[0], 0x00, p->_size);
  memset(p->_buffer[1], 0x00, p->_size);

  p->_index = 0;
  p->_length[0] = 0;
  p->_length[1] = 0;
  p->_readOffset[0] = 0;
  p->_readOffset[1] = 0;
}

size_t doubleBuffer_availableForWrite(struct PDMDoubleBuffer *p)
{
  return (p->_size - (p->_length[p->_index] - p->_readOffset[p->_index]));
}

size_t doubleBuffer_write(struct PDMDoubleBuffer *p, const void *buffer, size_t size)
{
  size_t space = doubleBuffer_availableForWrite(p);

  if (size > space) {
    size = space;
  }

  if (size == 0) {
    return 0;
  }

  memcpy(&(p->_buffer[p->_index][p->_length[p->_index]]), buffer, size);

  p->_length[p->_index] += size;

  return size;
}

size_t doubleBuffer_available(struct PDMDoubleBuffer *p)
{
  return p->_length[p->_index] - p->_readOffset[p->_index];
}

size_t doubleBuffer_read(struct PDMDoubleBuffer *p, void *buffer, size_t size)
{
  size_t avail = doubleBuffer_available(p);

  if (size > avail) {
    size = avail;
  }

  if (size == 0) {
    return 0;
  }

  memcpy(buffer, &(p->_buffer[p->_index][p->_readOffset[p->_index]]), size);
  p->_readOffset[p->_index] += size;

  return size;
}

void* doubleBuffer_data(struct PDMDoubleBuffer *p)
{
  return (void*)(p->_buffer[p->_index]);
}

void doubleBuffer_swap(struct PDMDoubleBuffer *p, int length)
{
  if (p->_index == 0) {
    p->_index = 1;
  } else {
    p->_index = 0;
  }

  p->_length[p->_index] = length;
  p->_readOffset[p->_index] = 0;
}

#define DEFAULT_PDM_GAIN     20
#define PDM_IRQ_PRIORITY     7

#define DEFAULT_PDM_BUFFER_SIZE 512
struct PDMDoubleBuffer doubleBuffer;
uint8_t _buffer[2][DEFAULT_PDM_BUFFER_SIZE];
uint8_t _channels;

#include <hal/nrf_gpio.h>

static int dmic_nrfx_pdm_configure_arduino(const struct device *dev,
				   struct dmic_cfg *config)
{
 	struct dmic_nrfx_pdm_drv_data *drv_data = dev->data;
	const struct dmic_nrfx_pdm_drv_cfg *drv_cfg = dev->config;
	struct pdm_chan_cfg *channel = &config->channel;
	struct pcm_stream_cfg *stream = &config->streams[0];
	nrfx_pdm_config_t nrfx_cfg;

  nrfx_cfg = drv_cfg->nrfx_def_cfg;

  // Enable high frequency oscillator if not already enabled
  if (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }
  }
  
  // configure the sample rate and channels
  switch (stream->pcm_rate) {
    case 16000:
      #ifndef NRF52832_XXAA
      NRF_PDM->RATIO = ((PDM_RATIO_RATIO_Ratio80 << PDM_RATIO_RATIO_Pos) & PDM_RATIO_RATIO_Msk);
      #endif
      nrf_pdm_clock_set(NRF_PDM, NRF_PDM_FREQ_1280K);
      break;
    default:
      return -1; // unsupported
  }

  switch (channel->req_num_chan) {
    case 2:
      nrf_pdm_mode_set(NRF_PDM, NRF_PDM_MODE_STEREO, NRF_PDM_EDGE_LEFTFALLING);
      break;

    case 1:
      nrf_pdm_mode_set(NRF_PDM, NRF_PDM_MODE_MONO, NRF_PDM_EDGE_LEFTFALLING);
      break;

    default:
      return -1; // unsupported
  }
  _channels = channel->req_num_chan;

  nrf_pdm_gain_set(NRF_PDM, DEFAULT_PDM_GAIN, DEFAULT_PDM_GAIN);

  // configure the I/O and mux
  nrf_gpio_cfg_output(nrfx_cfg.pin_clk);
  nrf_gpio_pin_clear(nrfx_cfg.pin_clk);

  nrf_gpio_cfg_input(nrfx_cfg.pin_din, NRF_GPIO_PIN_NOPULL);
  nrf_pdm_psel_connect(NRF_PDM, nrfx_cfg.pin_clk, nrfx_cfg.pin_din);

  // clear events and enable PDM interrupts
  nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_STARTED);
  nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_END);
  nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_STOPPED);
  nrf_pdm_int_enable(NRF_PDM, NRF_PDM_INT_STARTED | NRF_PDM_INT_STOPPED);

  // clear the buffer
  doubleBuffer_reset(&doubleBuffer, _buffer[0], _buffer[1], DEFAULT_PDM_BUFFER_SIZE);

  // set the PDM IRQ priority and enable
  NVIC_SetPriority(PDM_IRQn, PDM_IRQ_PRIORITY);
  NVIC_ClearPendingIRQ(PDM_IRQn);
  NVIC_EnableIRQ(PDM_IRQn);

  // enable and trigger start task
  nrf_pdm_enable(NRF_PDM);
  nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_STARTED);
  nrf_pdm_task_trigger(NRF_PDM, NRF_PDM_TASK_START);  
  
  return 0;
}

extern short sampleBuffer[256];
extern volatile int samplesRead;


void nrfx_pdm_irq_handler_arduino()
{
  if (nrf_pdm_event_check(NRF_PDM, NRF_PDM_EVENT_STARTED)) {
    nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_STARTED);

    if (doubleBuffer_available(&doubleBuffer) == 0) {
      // switch to the next buffer
      nrf_pdm_buffer_set(NRF_PDM, (uint32_t*)doubleBuffer_data(&doubleBuffer), doubleBuffer_availableForWrite(&doubleBuffer) / (sizeof(int16_t) * _channels));

      // make the current one available for reading
      doubleBuffer_swap(&doubleBuffer, doubleBuffer_availableForWrite(&doubleBuffer));

      // call receive callback if provided
      //if (_onReceive) {
      //  _onReceive();
      //}
      NVIC_DisableIRQ(PDM_IRQn);

      // query the number of bytes available
      int bytesAvailable = doubleBuffer_available(&doubleBuffer);

      // read into the sample buffer
      doubleBuffer_read(&doubleBuffer, sampleBuffer, bytesAvailable);

      // 16-bit, 2 bytes per sample
      samplesRead = bytesAvailable / 2;
      
      NVIC_EnableIRQ(PDM_IRQn);

    } else {
      // buffer overflow, stop
      nrf_pdm_disable(NRF_PDM);
    }
  } else if (nrf_pdm_event_check(NRF_PDM, NRF_PDM_EVENT_STOPPED)) {
    nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_STOPPED);
  } else if (nrf_pdm_event_check(NRF_PDM, NRF_PDM_EVENT_END)) {
    nrf_pdm_event_clear(NRF_PDM, NRF_PDM_EVENT_END);
  }
}

#endif // USE_ARDUINO_PDM

static int dmic_nrfx_pdm_configure(const struct device *dev,
				   struct dmic_cfg *config)
{
	struct dmic_nrfx_pdm_drv_data *drv_data = dev->data;
	const struct dmic_nrfx_pdm_drv_cfg *drv_cfg = dev->config;
	struct pdm_chan_cfg *channel = &config->channel;
	struct pcm_stream_cfg *stream = &config->streams[0];
	uint32_t def_map, alt_map;
	nrfx_pdm_config_t nrfx_cfg;
	nrfx_err_t err;

	if (drv_data->active) {
		LOG_ERR("Cannot configure device while it is active");
		return -EBUSY;
	}

	/*
	 * This device supports only one stream and can be configured to return
	 * 16-bit samples for two channels (Left+Right samples) or one channel
	 * (only Left samples). Left and Right samples can be optionally swapped
	 * by changing the PDM_CLK edge on which the sampling is done
	 * Provide the valid channel maps for both the above configurations
	 * (to inform the requester what is available) and check if what is
	 * requested can be actually configured.
	 */
	if (channel->req_num_chan == 1) {
		def_map = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT);
		alt_map = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT);

		channel->act_num_chan = 1;
	} else {
		def_map = dmic_build_channel_map(0, 0, PDM_CHAN_LEFT)
			| dmic_build_channel_map(1, 0, PDM_CHAN_RIGHT);
		alt_map = dmic_build_channel_map(0, 0, PDM_CHAN_RIGHT)
			| dmic_build_channel_map(1, 0, PDM_CHAN_LEFT);

		channel->act_num_chan = 2;
	}

	channel->act_num_streams = 1;
	channel->act_chan_map_hi = 0;
	channel->act_chan_map_lo = def_map;

	if (channel->req_num_streams != 1 ||
	    channel->req_num_chan > 2 ||
	    channel->req_num_chan < 1 ||
	    (channel->req_chan_map_lo != def_map &&
	     channel->req_chan_map_lo != alt_map) ||
	    channel->req_chan_map_hi != channel->act_chan_map_hi) {
		LOG_ERR("Requested configuration is not supported");
		return -EINVAL;
	}

	/* If either rate or width is 0, the stream is to be disabled. */
	if (stream->pcm_rate == 0 || stream->pcm_width == 0) {
		if (drv_data->configured) {
			nrfx_pdm_uninit();
			drv_data->configured = false;
		}

		return 0;
	}

	if (stream->pcm_width != 16) {
		LOG_ERR("Only 16-bit samples are supported");
		return -EINVAL;
	}

	nrfx_cfg = drv_cfg->nrfx_def_cfg;
	nrfx_cfg.mode = channel->req_num_chan == 1
		      ? NRF_PDM_MODE_MONO
		      : NRF_PDM_MODE_STEREO;
	nrfx_cfg.edge = channel->req_chan_map_lo == def_map
		      ? NRF_PDM_EDGE_LEFTFALLING
		      : NRF_PDM_EDGE_LEFTRISING;
#if NRF_PDM_HAS_MCLKCONFIG
	nrfx_cfg.mclksrc = drv_cfg->clk_src == ACLK
			 ? NRF_PDM_MCLKSRC_ACLK
			 : NRF_PDM_MCLKSRC_PCLK32M;
#endif
	if (!find_suitable_clock(drv_cfg, &nrfx_cfg, config)) {
		LOG_ERR("Cannot find suitable PDM clock configuration.");
		return -EINVAL;
	}

	if (drv_data->configured) {
		nrfx_pdm_uninit();
		drv_data->configured = false;
	}

  // FIXME: from Arduino. 
  // Enable high frequency oscillator if not already enabled
  if (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) {
    NRF_CLOCK->TASKS_HFCLKSTART    = 1;
    while (NRF_CLOCK->EVENTS_HFCLKSTARTED == 0) { }
  }
  nrfx_cfg.gain_l = 20;
  nrfx_cfg.gain_r = 20;
  // <<--

	err = nrfx_pdm_init(&nrfx_cfg, drv_cfg->event_handler);
	if (err != NRFX_SUCCESS) {
		LOG_ERR("Failed to initialize PDM: 0x%08x", err);
		return -EIO;
	}

	drv_data->block_size = stream->block_size;
	drv_data->mem_slab   = stream->mem_slab;

	/* Unless the PCLK32M source is used with the HFINT oscillator
	 * (which is always available without any additional actions),
	 * it is required to request the proper clock to be running
	 * before starting the transfer itself.
	 */
  // FIXME: commented out; do not wait for clock init 
  // - when this code is enabled, the driver never calls start_transfer() 
	//drv_data->request_clock = (drv_cfg->clk_src != PCLK32M); 
	drv_data->configured = true;
	return 0;
}

static int start_transfer(struct dmic_nrfx_pdm_drv_data *drv_data)
{
	nrfx_err_t err;
	int ret;

  printk("-- start_transfer\n");
	err = nrfx_pdm_start();
	if (err == NRFX_SUCCESS) {
		return 0;
	}

	LOG_ERR("Failed to start PDM: 0x%08x", err);
	ret =  -EIO;

	if (drv_data->request_clock) {
		(void)onoff_release(drv_data->clk_mgr);
	}

	drv_data->active = false;
	return ret;
}

static void clock_started_callback(struct onoff_manager *mgr,
				   struct onoff_client *cli,
				   uint32_t state,
				   int res)
{
	struct dmic_nrfx_pdm_drv_data *drv_data =
		CONTAINER_OF(cli, struct dmic_nrfx_pdm_drv_data, clk_cli);

	/* The driver can turn out to be inactive at this point if the STOP
	 * command was triggered before the clock has started. Do not start
	 * the actual transfer in such case.
	 */
	if (!drv_data->active) {
		(void)onoff_release(drv_data->clk_mgr);
	} else {
		(void)start_transfer(drv_data);
	}
}

static int trigger_start(const struct device *dev)
{
	struct dmic_nrfx_pdm_drv_data *drv_data = dev->data;
	int ret;

	drv_data->active = true;

	/* If it is required to use certain HF clock, request it to be running
	 * first. If not, start the transfer directly.
	 */
	if (drv_data->request_clock) {
    printk("trigger_start 1\n");
		sys_notify_init_callback(&drv_data->clk_cli.notify,
					 clock_started_callback);
		ret = onoff_request(drv_data->clk_mgr, &drv_data->clk_cli);
		if (ret < 0) {
			drv_data->active = false;

			LOG_ERR("Failed to request clock: %d", ret);
			return -EIO;
		}
	} else {
    printk("trigger_start 2\n");
		ret = start_transfer(drv_data);
		if (ret < 0) {
			return ret;
		}
	}

	return 0;
}

static int dmic_nrfx_pdm_trigger(const struct device *dev,
				 enum dmic_trigger cmd)
{
	struct dmic_nrfx_pdm_drv_data *drv_data = dev->data;

	switch (cmd) {
	case DMIC_TRIGGER_PAUSE:
	case DMIC_TRIGGER_STOP:
		if (drv_data->active) {
			nrfx_pdm_stop();
			drv_data->stopping = true;
		}
		break;

	case DMIC_TRIGGER_RELEASE:
	case DMIC_TRIGGER_START:
		if (!drv_data->configured) {
			LOG_ERR("Device is not configured");
			return -EIO;
		} else if (!drv_data->active) {
			drv_data->stopping = false;
			return trigger_start(dev);
		}
		break;

	default:
		LOG_ERR("Invalid command: %d", cmd);
		return -EINVAL;
	}

	return 0;
}

static int dmic_nrfx_pdm_read(const struct device *dev,
			      uint8_t stream,
			      void **buffer, size_t *size, int32_t timeout)
{
	struct dmic_nrfx_pdm_drv_data *drv_data = dev->data;
	int ret;

	ARG_UNUSED(stream);

	if (!drv_data->configured) {
		LOG_ERR("Device is not configured");
		return -EIO;
	}

	ret = k_msgq_get(&drv_data->rx_queue, buffer, SYS_TIMEOUT_MS(timeout));
	if (ret != 0) {
		LOG_ERR("No audio data to be read");
	} else {
		LOG_DBG("Released buffer %p", *buffer);

		*size = drv_data->block_size;
	}

	return ret;
}

static void init_clock_manager(const struct device *dev)
{
	struct dmic_nrfx_pdm_drv_data *drv_data = dev->data;
	clock_control_subsys_t subsys;

#if NRF_CLOCK_HAS_HFCLKAUDIO
	const struct dmic_nrfx_pdm_drv_cfg *drv_cfg = dev->config;

	if (drv_cfg->clk_src == ACLK) {
		subsys = CLOCK_CONTROL_NRF_SUBSYS_HFAUDIO;
	} else
#endif
	{
		subsys = CLOCK_CONTROL_NRF_SUBSYS_HF;
	}

	drv_data->clk_mgr = z_nrf_clock_control_get_onoff(subsys);
	__ASSERT_NO_MSG(drv_data->clk_mgr != NULL);
}

static const struct _dmic_ops dmic_ops = {
#if defined(USE_NRFX_PDM)
	.configure = dmic_nrfx_pdm_configure,
#elif defined(USE_ARDUINO_PDM)
	.configure = dmic_nrfx_pdm_configure_arduino,
#endif
	.trigger = dmic_nrfx_pdm_trigger,
	.read = dmic_nrfx_pdm_read,
};

#if defined(USE_NRFX_PDM)
  #define NRFS_PDM_IRQ_HANDLER nrfx_pdm_irq_handler
#elif defined(USE_ARDUINO_PDM)
  #define NRFS_PDM_IRQ_HANDLER nrfx_pdm_irq_handler_arduino
#endif

#define PDM(idx) DT_NODELABEL(pdm##idx)
#define PDM_CLK_SRC(idx) DT_STRING_TOKEN(PDM(idx), clock_source)

#define PDM_NRFX_DEVICE(idx)						     \
	static void *rx_msgs##idx[DT_PROP(PDM(idx), queue_size)];	     \
	static struct dmic_nrfx_pdm_drv_data dmic_nrfx_pdm_data##idx;	     \
	static int pdm_nrfx_init##idx(const struct device *dev)		     \
	{								     \
		IRQ_CONNECT(DT_IRQN(PDM(idx)), DT_IRQ(PDM(idx), priority),   \
			    nrfx_isr, NRFS_PDM_IRQ_HANDLER, 0);		     \
		irq_enable(DT_IRQN(PDM(idx)));				     \
		k_msgq_init(&dmic_nrfx_pdm_data##idx.rx_queue,		     \
			    (char *)rx_msgs##idx, sizeof(void *),	     \
			    ARRAY_SIZE(rx_msgs##idx));			     \
		init_clock_manager(dev);				     \
		return 0;						     \
	}								     \
	static void event_handler##idx(const nrfx_pdm_evt_t *evt)	     \
	{								     \
		event_handler(DEVICE_DT_GET(PDM(idx)), evt);		     \
	}								     \
	static const struct dmic_nrfx_pdm_drv_cfg dmic_nrfx_pdm_cfg##idx = { \
		.event_handler = event_handler##idx,			     \
		.nrfx_def_cfg =	NRFX_PDM_DEFAULT_CONFIG(		     \
					DT_PROP(PDM(idx), clk_pin),	     \
					DT_PROP(PDM(idx), din_pin)),	     \
		.clk_src = PDM_CLK_SRC(idx),				     \
	};								     \
	BUILD_ASSERT(PDM_CLK_SRC(idx) != ACLK || NRF_PDM_HAS_MCLKCONFIG,     \
		"Clock source ACLK is not available.");			     \
	BUILD_ASSERT(PDM_CLK_SRC(idx) != ACLK ||			     \
		     DT_NODE_HAS_PROP(DT_NODELABEL(clock),		     \
				      hfclkaudio_frequency),		     \
		"Clock source ACLK requires the hfclkaudio-frequency "	     \
		"property to be defined in the nordic,nrf-clock node.");     \
	DEVICE_DT_DEFINE(PDM(idx), pdm_nrfx_init##idx, NULL,		     \
			 &dmic_nrfx_pdm_data##idx, &dmic_nrfx_pdm_cfg##idx,  \
			 POST_KERNEL, CONFIG_AUDIO_DMIC_INIT_PRIORITY,	     \
			 &dmic_ops);

/* Existing SoCs only have one PDM instance. */
PDM_NRFX_DEVICE(0);
