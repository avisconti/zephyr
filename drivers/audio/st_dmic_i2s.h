/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef ST_DMIC_I2S_H
#define ST_DMIC_I2S_H

#ifdef __cplusplus
extern "C" {
#endif

#include <audio/dmic.h>
#include <zephyr.h>
#include <i2s.h>
#include <device.h>

#define ST_DMIC_I2S_MIN_PDM_FREQ		1200000 /* 1.2MHz */
#define ST_DMIC_I2S_MAX_PDM_FREQ		3250000 /* 3.25MHz */

#define DEV_CFG(dev) \
	((struct st_dmic_i2s_driver_config *const)(dev)->config->config_info)
#define DEV_DATA(dev) \
	((struct st_dmic_i2s_driver_data *const)(dev)->driver_data)

struct st_dmic_i2s_driver_config {
	struct device	*i2s_device;
};

#ifdef __cplusplus
}
#endif

#endif /* ST_DMIC_I2S_H */
