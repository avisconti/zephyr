/* ST Microelectronics LIS2DUX12 smart accelerometer APIs
 *
 * Copyright (c) 2024 STMicroelectronics
 * Copyright (c) 2023 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lis2dux12.h"
#include "lis2dux12_api.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(LIS2DUX12, CONFIG_SENSOR_LOG_LEVEL);

static inline int lis2dux12_set_odr_raw(const struct device *dev, uint8_t odr)
{
	struct lis2dux12_data *data = dev->data;
	const struct lis2dux12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lis2dux12_md_t mode = {.odr = odr, .fs = data->range};

	data->odr = odr;
	return lis2dux12_mode_set(ctx, &mode);
}

static inline int lis2dux12_set_range(const struct device *dev, uint8_t range)
{
	int err;
	struct lis2dux12_data *data = dev->data;
	const struct lis2dux12_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lis2dux12_md_t val = { .odr = data->odr, .fs = range };

	err = lis2dux12_mode_set(ctx, &val);

	if (err) {
		return err;
	}

	switch (range) {
	default:
		LOG_ERR("range [%d] not supported.", range);
		return -EINVAL;
	case LIS2DUX12_DT_FS_2G:
		data->gain = lis2dux12_from_fs2g_to_mg(1);
		break;
	case LIS2DUX12_DT_FS_4G:
		data->gain = lis2dux12_from_fs4g_to_mg(1);
		break;
	case LIS2DUX12_DT_FS_8G:
		data->gain = lis2dux12_from_fs8g_to_mg(1);
		break;
	case LIS2DUX12_DT_FS_16G:
		data->gain = lis2dux12_from_fs16g_to_mg(1);
		break;
	}

	data->range = range;
	return 0;
}

const struct lis2dux12_chip_api st_lis2dux12_chip_api = {
	.set_odr_raw = lis2dux12_set_odr_raw,
	.set_range = lis2dux12_set_range,
};

#if 0
int st_lps22df_init(const struct device *dev)
{
	const struct lps2xdf_config *const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps22df_id_t id;
	lps22df_stat_t status;
	uint8_t tries = 10;
	int ret;

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lps22df, i3c)
	if (cfg->i3c.bus != NULL) {
		struct lps2xdf_data *data = dev->data;
		/*
		 * Need to grab the pointer to the I3C device descriptor
		 * before we can talk to the sensor.
		 */
		data->i3c_dev = i3c_device_find(cfg->i3c.bus, &cfg->i3c.dev_id);
		if (data->i3c_dev == NULL) {
			LOG_ERR("Cannot find I3C device descriptor");
			return -ENODEV;
		}
	}
#endif

	if (lps22df_id_get(ctx, &id) < 0) {
		LOG_ERR("%s: Not able to read dev id", dev->name);
		return -EIO;
	}

	if (id.whoami != LPS22DF_ID) {
		LOG_ERR("%s: Invalid chip ID 0x%02x", dev->name, id.whoami);
		return -EIO;
	}

	LOG_DBG("%s: chip id 0x%x", dev->name, id.whoami);

	/* Restore default configuration */
	if (lps22df_init_set(ctx, LPS22DF_RESET) < 0) {
		LOG_ERR("%s: Not able to reset device", dev->name);
		return -EIO;
	}

	do {
		if (!--tries) {
			LOG_DBG("sw reset timed out");
			return -ETIMEDOUT;
		}
		k_usleep(LPS2XDF_SWRESET_WAIT_TIME_US);

		if (lps22df_status_get(ctx, &status) < 0) {
			return -EIO;
		}
	} while (status.sw_reset);

	/* Set bdu and if_inc recommended for driver usage */
	if (lps22df_init_set(ctx, LPS22DF_DRV_RDY) < 0) {
		LOG_ERR("%s: Not able to set device to ready state", dev->name);
		return -EIO;
	}

	if (ON_I3C_BUS(cfg)) {
		lps22df_bus_mode_t bus_mode;

		/* Select bus interface */
		lps22df_bus_mode_get(ctx, &bus_mode);
		bus_mode.filter = LPS22DF_AUTO;
		bus_mode.interface = LPS22DF_SEL_BY_HW;
		lps22df_bus_mode_set(ctx, &bus_mode);
	}

	/* set sensor default odr */
	LOG_DBG("%s: odr: %d", dev->name, cfg->odr);
	ret = lps22df_set_odr_raw(dev, cfg->odr);
	if (ret < 0) {
		LOG_ERR("%s: Failed to set odr %d", dev->name, cfg->odr);
		return ret;
	}

#ifdef CONFIG_LPS2XDF_TRIGGER
	if (cfg->trig_enabled) {
		if (lps2xdf_init_interrupt(dev, DEVICE_VARIANT_LPS22DF) < 0) {
			LOG_ERR("Failed to initialize interrupt.");
			return -EIO;
		}
	}
#endif

	return 0;
}
#endif
