/*
 * Copyright (c) 2019 Peter Bigot Consulting, LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/* Sample which uses the filesystem API with littlefs */

#include <stdio.h>

#include <zephyr.h>
#include <device.h>
#include <fs/fs.h>
#include <fs/littlefs.h>
#include <storage/flash_map.h>
// Including the libraries that I think are going to help me with the 
#include <sys/printk.h>
#include <sys/util.h>
#include <string.h>
#include <usb/usb_device.h>
#include <drivers/uart.h>

#include <audio/dmic.h>

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

/* Matches LFS_NAME_MAX */
#define MAX_PATH_LEN 255

#define PARTITION_NODE DT_NODELABEL(lfs1)

#if DT_NODE_EXISTS(PARTITION_NODE)
FS_FSTAB_DECLARE_ENTRY(PARTITION_NODE);
#else /* PARTITION_NODE */
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(storage);
static struct fs_mount_t lfs_storage_mnt = {
	.type = FS_LITTLEFS,
	.fs_data = &storage,
	.storage_dev = (void *)FLASH_AREA_ID(storage),
	.mnt_point = "/lfs",
};
#endif /* PARTITION_NODE */

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

	struct fs_mount_t *mp =
#if DT_NODE_EXISTS(PARTITION_NODE)
		&FS_FSTAB_ENTRY(PARTITION_NODE)
#else
		&lfs_storage_mnt
#endif
		;
	unsigned int id = (uintptr_t)mp->storage_dev;
	char fname[MAX_PATH_LEN];
	struct fs_statvfs sbuf;
	const struct flash_area *pfa;
	int rc;

	snprintf(fname, sizeof(fname), "%s/boot_count", mp->mnt_point);

	rc = flash_area_open(id, &pfa);
	if (rc < 0) {
		printk("FAIL: unable to find flash area %u: %d\n",
		       id, rc);
		return;
	}

	printk("Area %u at 0x%x on %s for %u bytes\n",
	       id, (unsigned int)pfa->fa_off, pfa->fa_dev_name,
	       (unsigned int)pfa->fa_size);

	/* Optional wipe flash contents */
	if (IS_ENABLED(CONFIG_APP_WIPE_STORAGE)) {
		printk("Erasing flash area ... ");
		rc = flash_area_erase(pfa, 0, pfa->fa_size);
		printk("%d\n", rc);
	}

	flash_area_close(pfa);

	/* Do not mount if auto-mount has been enabled */
#if !DT_NODE_EXISTS(PARTITION_NODE) ||						\
	!(FSTAB_ENTRY_DT_MOUNT_FLAGS(PARTITION_NODE) & FS_MOUNT_FLAG_AUTOMOUNT)
	rc = fs_mount(mp);
	if (rc < 0) {
		printk("FAIL: mount id %u at %s: %d\n",
		       (unsigned int)mp->storage_dev, mp->mnt_point,
		       rc);
		return;
	}
	printk("%s mount: %d\n", mp->mnt_point, rc);
#else
	printk("%s automounted\n", mp->mnt_point);
#endif

	rc = fs_statvfs(mp->mnt_point, &sbuf);
	if (rc < 0) {
		printk("FAIL: statvfs: %d\n", rc);
		goto out;
	}

	printk("%s: bsize = %lu ; frsize = %lu ;"
	       " blocks = %lu ; bfree = %lu\n",
	       mp->mnt_point,
	       sbuf.f_bsize, sbuf.f_frsize,
	       sbuf.f_blocks, sbuf.f_bfree);

	struct fs_dirent dirent;

	rc = fs_stat(fname, &dirent);
	printk("%s stat: %d\n", fname, rc);
	if (rc >= 0) {
		printk("\tfn '%s' siz %u\n", dirent.name, dirent.size);
	}

	// This is the WAV Header format
	char chunkID[4] = {'R', 'I', 'F', 'F'};
	uint32_t chunkSize = 36;
	char format[4] = {'W', 'A', 'V', 'E'};
	char subChunk1ID[4] = {'f', 'm', 't', ' '};
	uint32_t subChunk1Size = 16;
	uint16_t audioFormat = 1;
	uint16_t numChannels = 1;
	uint32_t sampleRate = 44100;
	uint32_t byteRate = 44100 * 2;
	uint16_t blockAlign = 2;
	uint16_t bitsPerSample = 16;
	char subChunk2ID[4] = {'d', 'a', 't', 'a'};
	uint32_t subChunk2Size = 0;
	int boot_count = 0;
	struct fs_file_t file;

	fs_file_t_init(&file);

	rc = fs_open(&file, fname, FS_O_CREATE | FS_O_RDWR);
	if (rc < 0) {
		printk("FAIL: open %s: %d\n", fname, rc);
		goto out;
	}



	if (rc >= 0) {
		rc = fs_read(&file, &boot_count, sizeof(boot_count));
		printk("%s read count %u: %d\n", fname, boot_count, rc);
		rc = fs_seek(&file, 0, FS_SEEK_SET);
		printk("%s seek start: %d\n", fname, rc);

	}

	// boot_count += 1;
	rc = fs_write(&file, chunkID, sizeof(chunkID));
	rc = fs_write(&file, &chunkSize, sizeof(chunkSize));
	rc = fs_write(&file, format, sizeof(format));
	rc = fs_write(&file, subChunk1ID, sizeof(subChunk1ID));
	rc = fs_write(&file, &subChunk1Size, sizeof(subChunk1Size));
	rc = fs_write(&file, &audioFormat, sizeof(audioFormat));
	rc = fs_write(&file, &numChannels, sizeof(numChannels));
	rc = fs_write(&file, &sampleRate, sizeof(sampleRate));
	rc = fs_write(&file, &byteRate, sizeof(byteRate));
	rc = fs_write(&file, &blockAlign, sizeof(blockAlign));
	rc = fs_write(&file, &bitsPerSample, sizeof(bitsPerSample));
	rc = fs_write(&file, subChunk2ID, sizeof(subChunk2ID));
	rc = fs_write(&file, &subChunk2Size, sizeof(subChunk2Size));
	printk("File writing is complete");
	// Write Header and Data over here
	rc = fs_close(&file);
	printk("%s close: %d\n", fname, rc);

out:
	rc = fs_unmount(mp);
	printk("%s unmount: %d\n", mp->mnt_point, rc);
}
