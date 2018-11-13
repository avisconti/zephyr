/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "st_dmic_i2s.h"
#include "OpenPDMFilter.h"

#define LOG_LEVEL CONFIG_ST_DMIC_I2S_LOG_LEVEL
#include <logging/log.h>
LOG_MODULE_REGISTER(st_dmic_i2s);

#define NUM_RX_BLOCKS			4
#define PDM_BLOCK_MAX_SIZE_BYTES	512

K_MEM_SLAB_DEFINE(rx_pdm_mem_slab, PDM_BLOCK_MAX_SIZE_BYTES, NUM_RX_BLOCKS, 1);

/* ST_DMIC_I2S private data */
static struct st_dmic_i2s_pdata {
	enum dmic_state state;
	TPDMFilter_InitStruct pdm_filter;
	size_t pcm_mem_size;
	struct k_mem_slab *pcm_mem_slab;
} st_dmic_i2s_private;

static int st_dmic_i2s_read(struct device *dev, u8_t stream, void **buffer,
			 size_t *size, s32_t timeout)
{
	int i, ret;
	struct st_dmic_i2s_driver_config *const dev_cfg = DEV_CFG(dev);
	void *pdm_block, *pcm_block;
	size_t pdm_size;
	TPDMFilter_InitStruct *pdm_filter = &st_dmic_i2s_private.pdm_filter;

	ret = i2s_read(dev_cfg->i2s_device, &pdm_block, &pdm_size);
	if (ret != 0) {
		LOG_ERR("read failed (%d)", ret);
		return ret;
	}

	for (i = 0; i < pdm_size/2; i++) {
		((u16_t *)pdm_block)[i] = HTONS(((u16_t *)pdm_block)[i]);
	}

	ret = k_mem_slab_alloc(st_dmic_i2s_private.pcm_mem_slab,
			       &pcm_block, K_NO_WAIT);
	if (ret < 0) {
		return ret;
	}

	switch (pdm_filter->Decimation) {
	case 64:
		Open_PDM_Filter_64((u8_t *) pdm_block, pcm_block,
				    st_dmic_i2s_private.pcm_mem_size, pdm_filter);
		break;
	case 128:
		Open_PDM_Filter_128((u8_t *) pdm_block, pcm_block,
				    st_dmic_i2s_private.pcm_mem_size, pdm_filter);
		break;
	default:
		return -EINVAL;
	}
	k_mem_slab_free(&rx_pdm_mem_slab, &pdm_block);

	*buffer = pcm_block;
	*size = st_dmic_i2s_private.pcm_mem_size;

	return 0;
}

static int st_dmic_i2s_trigger(struct device *dev, enum dmic_trigger cmd)
{
	int ret;
	struct st_dmic_i2s_driver_config *const dev_cfg = DEV_CFG(dev);
	enum i2s_trigger_cmd i2s_cmd;
	enum dmic_state tmp_state;

	switch (cmd) {
	case DMIC_TRIGGER_START:
		if (st_dmic_i2s_private.state == DMIC_STATE_CONFIGURED) {
			tmp_state = DMIC_STATE_ACTIVE;
			i2s_cmd = I2S_TRIGGER_START;
		} else {
			return 0;
		}
		break;
	case DMIC_TRIGGER_STOP:
		if (st_dmic_i2s_private.state == DMIC_STATE_ACTIVE) {
			tmp_state = DMIC_STATE_CONFIGURED;
			i2s_cmd = I2S_TRIGGER_STOP;
		} else {
			return 0;
		}
		break;
	default:
		return -EINVAL;
	}

	ret = i2s_trigger(dev_cfg->i2s_device, I2S_DIR_RX, i2s_cmd);
	if (ret != 0) {
		LOG_ERR("trigger failed with %d error", ret);
		return ret;
	}

	st_dmic_i2s_private.state = tmp_state;
	return 0;
}

static int st_dmic_i2s_configure(struct device *dev, struct dmic_cfg *cfg)
{
	int ret;
	struct st_dmic_i2s_driver_config *const dev_cfg = DEV_CFG(dev);
	u8_t chan_size = cfg->streams->pcm_width;
	u32_t audio_freq = cfg->streams->pcm_rate;
	TPDMFilter_InitStruct *pdm_filter = &st_dmic_i2s_private.pdm_filter;
	u16_t factor;

	/* PCM buffer size */
	st_dmic_i2s_private.pcm_mem_slab = cfg->streams->mem_slab;
	st_dmic_i2s_private.pcm_mem_size = cfg->streams->block_size;

	/* check requested min pdm frequency */
	if (cfg->io.min_pdm_clk_freq < ST_DMIC_I2S_MIN_PDM_FREQ ||
	    cfg->io.min_pdm_clk_freq > cfg->io.max_pdm_clk_freq) {
		return -EINVAL;
	}

	/* check requested max pdm frequency */
	if (cfg->io.max_pdm_clk_freq > ST_DMIC_I2S_MAX_PDM_FREQ ||
	    cfg->io.max_pdm_clk_freq < cfg->io.min_pdm_clk_freq) {
		return -EINVAL;
	}

	/* calculate oversampling factor based on pdm clock */
	for (factor = 64; factor <= 128; factor += 64) {
		u32_t pdm_bit_clk = (audio_freq * factor * \
				     cfg->channel.req_num_chan);

		if (pdm_bit_clk >= cfg->io.min_pdm_clk_freq &&
		    pdm_bit_clk <= cfg->io.max_pdm_clk_freq) {
			break;
		}
	}

	if (factor != 64 && factor != 128) {
		return -EINVAL;
	}

	/* init the filter lib */
	pdm_filter->LP_HZ = audio_freq / 2;
	pdm_filter->HP_HZ = 10;
	pdm_filter->Fs = audio_freq;
	pdm_filter->Out_MicChannels = 1;
	pdm_filter->In_MicChannels = 1;
	pdm_filter->Decimation = factor;
	pdm_filter->MaxVolume = 64;

	Open_PDM_Filter_Init(pdm_filter);

	/* configure I2S channels */
	struct i2s_config i2s_cfg;

	i2s_cfg.word_size = chan_size;
	i2s_cfg.channels = cfg->channel.req_num_chan;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED |
			 I2S_FMT_BIT_CLK_INV;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.frame_clk_freq = audio_freq * factor / chan_size;
	i2s_cfg.block_size = st_dmic_i2s_private.pcm_mem_size * (factor / chan_size);
	i2s_cfg.mem_slab = &rx_pdm_mem_slab;
	i2s_cfg.timeout = 2000;

	ret = i2s_configure(dev_cfg->i2s_device, I2S_DIR_RX, &i2s_cfg);
	if (ret != 0) {
		LOG_ERR("I2S device configuration error");
		return ret;
	}

	st_dmic_i2s_private.state = DMIC_STATE_CONFIGURED;
	return 0;
}

static const struct _dmic_ops st_dmic_i2s_driver_api = {
	.configure		= st_dmic_i2s_configure,
	.trigger		= st_dmic_i2s_trigger,
	.read			= st_dmic_i2s_read,
};

static int st_dmic_i2s_initialize(struct device *dev)
{
	struct st_dmic_i2s_driver_config *const dev_cfg = DEV_CFG(dev);

	/* bind I2S */
	dev_cfg->i2s_device = device_get_binding(DT_ST_DMIC_I2S_MASTER_DEV_NAME);

	if (dev_cfg->i2s_device == NULL) {
		LOG_ERR("I2S device binding error");
		return -ENXIO;
	}

	st_dmic_i2s_private.state = DMIC_STATE_INITIALIZED;
	return 0;
}

static struct st_dmic_i2s_driver_config st_dmic_i2s_drv_cfg;

DEVICE_AND_API_INIT(st_dmic_i2s, DT_ST_DMIC_I2S_DEV_NAME, st_dmic_i2s_initialize,
		NULL, &st_dmic_i2s_drv_cfg, POST_KERNEL,
		CONFIG_AUDIO_DMIC_INIT_PRIORITY, &st_dmic_i2s_driver_api);
