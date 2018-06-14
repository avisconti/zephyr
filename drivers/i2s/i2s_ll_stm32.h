/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef _STM32_I2S_H_
#define _STM32_I2S_H_

#ifdef CONFIG_SOC_SERIES_STM32F4X
#define I2S1_DMA_NAME		CONFIG_DMA_2_NAME
#define I2S1_DMA_CHAN_RX	2
#define I2S1_DMA_SLOT_RX	3
#define I2S1_DMA_CHAN_TX	3
#define I2S1_DMA_SLOT_TX	3
#define I2S2_DMA_NAME		CONFIG_DMA_1_NAME
#define I2S2_DMA_CHAN_RX	3
#define I2S2_DMA_SLOT_RX	0
#define I2S2_DMA_CHAN_TX	4
#define I2S2_DMA_SLOT_TX	0
#define I2S3_DMA_NAME		CONFIG_DMA_1_NAME
#define I2S3_DMA_CHAN_RX	0
#define I2S3_DMA_SLOT_RX	0
#define I2S3_DMA_CHAN_TX	5
#define I2S3_DMA_SLOT_TX	0
#define I2S4_DMA_NAME		CONFIG_DMA_2_NAME
#define I2S4_DMA_CHAN_RX	0
#define I2S4_DMA_SLOT_RX	4
#define I2S4_DMA_CHAN_TX	1
#define I2S4_DMA_SLOT_TX	4
#define I2S5_DMA_NAME		CONFIG_DMA_2_NAME
#define I2S5_DMA_CHAN_RX	5
#define I2S5_DMA_SLOT_RX	7
#define I2S5_DMA_CHAN_TX	6
#define I2S5_DMA_SLOT_TX	7
#endif

#define DEV_CFG(dev) \
	(const struct i2s_stm32_cfg * const)((dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct i2s_stm32_data *const)(dev)->driver_data)

struct queue_item {
	void *mem_block;
	size_t size;
};

/* Minimal ring buffer implementation */
struct ring_buf {
	struct queue_item *buf;
	u16_t len;
	u16_t head;
	u16_t tail;
};

/* Device constant configuration parameters */
struct i2s_stm32_cfg {
	SPI_TypeDef *i2s;
	struct stm32_pclken pclken;
	u32_t i2s_clk_sel;
	void (*irq_config)(struct device *dev);
};

struct stream {
	s32_t state;
	struct k_sem sem;
	u32_t dma_channel;
	struct dma_config dma_cfg;
	struct i2s_config cfg;
	struct ring_buf mem_block_queue;
	void *mem_block;
	bool last_block;
	bool master;
	int (*stream_start)(struct stream *, struct device *dev);
	void (*stream_disable)(struct stream *, struct device *dev);
	void (*queue_drop)(struct stream *);
};

/* Device run time data */
struct i2s_stm32_data {
	struct device *dev_dma;
	const char *dma_name;
	struct stream rx;
	struct stream tx;
};

#endif	/* _STM32_I2S_H_ */
