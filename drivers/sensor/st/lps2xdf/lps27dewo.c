/* ST Microelectronics LPS27DEWO pressure and temperature sensor
 *
 * Copyright (c) 2025 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "lps2xdf.h"
#include "lps27dewo.h"
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(LPS2XDF, CONFIG_SENSOR_LOG_LEVEL);

static inline void lps27dewo_press_convert(const struct device *dev,
					   struct sensor_value *val,
					   int32_t raw_val)
{
	const struct lps2xdf_config *const cfg = dev->config;
	int32_t press_tmp = raw_val >> 8; /* raw value is left aligned (24 msb) */
	int divider;

	/* Pressure sensitivity is:
	 * - 2700 LSB/hPa for Full-Scale of 1260 hPa:
	 * - 1350 LSB/hPa for Full-Scale of 6060 hPa:
	 * Also convert hPa into kPa
	 */
	if (cfg->fs == 0) {
		/* Mode 1 (fs is 1260 hPa) */
		divider = 27000;
	} else {
		/* Mode 2 (fs is 6060 hPa) */
		divider = 13500;
	}
	val->val1 = press_tmp / divider;

	/* For the decimal part use (1000 / 27) as a factor instead of
	 * (1000000 / 27000) to avoid int32 overflow
	 */
	val->val2 = (press_tmp % divider) * 1000 / 27;
}

static inline void lps27dewo_temp_convert(const struct device *dev,
					  struct sensor_value *val,
					  int16_t raw_val)
{
	/* Temperature sensitivity is 100 LSB/deg C */
	val->val1 = raw_val / 100;
	val->val2 = ((int32_t)raw_val % 100) * 10000;
}

static inline int lps27dewo_mode_set_odr_raw(const struct device *dev, uint8_t odr)
{
	struct lps2xdf_data *data = dev->data;
	const struct lps2xdf_config *const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps27dewo_md_t md;
	int ret;

	md.odr = odr;
	md.avg = cfg->avg;
	md.lpf = cfg->lpf;
	md.fs = cfg->fs;

	ret = lps27dewo_mode_set(ctx, &md);
	if (ret == 0) {
		data->pag = md.comp.pag;
		data->Delta_P0 = md.comp.Delta_P0;
		data->dgain = md.comp.dgain;
		data->knl2 = md.comp.knl2;
		data->knl3 = md.comp.knl3;
	}

	return ret;
}

static int lps27dewo_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
	struct lps2xdf_data *data = dev->data;
	const struct lps2xdf_config *const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps27dewo_data_t raw_data;
	lps27dewo_md_t md;

	md.fs = cfg->fs;
	md.comp.pag = data->pag;
	md.comp.Delta_P0 = data->Delta_P0;
	md.comp.dgain = data->dgain;
	md.comp.knl2 = data->knl2;
	md.comp.knl3 = data->knl3;

	if (lps27dewo_data_get(ctx, &md, &raw_data) < 0) {
		LOG_DBG("Failed to read sample");
		return -EIO;
	}

	data->sample_press = raw_data.pressure.raw;
	data->sample_temp = raw_data.heat.raw;

	return 0;
}

#ifdef CONFIG_LPS2XDF_TRIGGER
/**
 * lps27dewo_config_interrupt - config the interrupt mode
 */
static int lps27dewo_config_interrupt(const struct device *dev)
{
	const struct lps2xdf_config *const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps27dewo_int_mode_t mode;

	if (lps27dewo_interrupt_mode_get(ctx, &mode) < 0) {
		return -EIO;
	}

	mode.drdy_latched = ~cfg->drdy_pulsed;

	return lps27dewo_interrupt_mode_set(ctx, &mode);
}

/**
 * lps27dewo_handle_interrupt - handle the drdy event
 * read data and call handler if registered any
 */
static void lps27dewo_handle_interrupt(const struct device *dev)
{
	int ret;
	struct lps2xdf_data *lps27dewo = dev->data;
	const struct lps2xdf_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps27dewo_all_sources_t status;

	if (lps27dewo_all_sources_get(ctx, &status) < 0) {
		LOG_DBG("failed reading status reg");
		goto exit;
	}

	if (status.drdy_pres == 0) {
		goto exit; /* spurious interrupt */
	}

	if (lps27dewo->handler_drdy != NULL) {
		lps27dewo->handler_drdy(dev, lps27dewo->data_ready_trigger);
	}

	if (ON_I3C_BUS(cfg)) {
		/*
		 * I3C IBI does not rely on GPIO.
		 * So no need to enable GPIO pin for interrupt trigger.
		 */
		return;
	}

exit:
	ret = gpio_pin_interrupt_configure_dt(&cfg->gpio_int,
					      GPIO_INT_EDGE_TO_ACTIVE);
	if (ret < 0) {
		LOG_ERR("%s: Not able to configure pin_int", dev->name);
	}
}

/**
 * lps27dewo_enable_int - enable selected int pin to generate interrupt
 */
static int lps27dewo_enable_int(const struct device *dev, int enable)
{
	const struct lps2xdf_config * const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps27dewo_pin_int_route_t int_route;

	/* set interrupt */
	lps27dewo_pin_int_route_get(ctx, &int_route);
	int_route.drdy_pres = enable;
	return lps27dewo_pin_int_route_set(ctx, &int_route);
}

/**
 * lps27dewo_trigger_set - link external trigger to event data ready
 */
static int lps27dewo_trigger_set(const struct device *dev,
			  const struct sensor_trigger *trig,
			  sensor_trigger_handler_t handler)
{
	struct lps2xdf_data *lps27dewo = dev->data;
	const struct lps2xdf_config * const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps27dewo_data_t raw_data;
	lps27dewo_md_t md;

	md.fs = cfg->fs;

	if (trig->chan != SENSOR_CHAN_ALL) {
		LOG_WRN("trigger set not supported on this channel.");
		return -ENOTSUP;
	}

	lps27dewo->handler_drdy = handler;
	lps27dewo->data_ready_trigger = trig;
	if (handler) {
		/* dummy read: re-trigger interrupt */
		if (lps27dewo_data_get(ctx, &md, &raw_data) < 0) {
			LOG_DBG("Failed to read sample");
			return -EIO;
		}
		return lps27dewo_enable_int(dev, 1);
	} else {
		return lps27dewo_enable_int(dev, 0);
	}

	return -ENOTSUP;
}
#endif /* CONFIG_LPS2XDF_TRIGGER */

const struct lps2xdf_chip_api st_lps27dewo_chip_api = {
	.mode_set_odr_raw = lps27dewo_mode_set_odr_raw,
	.sample_fetch = lps27dewo_sample_fetch,
	.press_convert = lps27dewo_press_convert,
	.temp_convert = lps27dewo_temp_convert,
#if CONFIG_LPS2XDF_TRIGGER
	.config_interrupt = lps27dewo_config_interrupt,
	.handle_interrupt = lps27dewo_handle_interrupt,
	.trigger_set = lps27dewo_trigger_set,
#endif
};

int st_lps27dewo_init(const struct device *dev)
{
	const struct lps2xdf_config *const cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	lps27dewo_id_t id;
	lps27dewo_stat_t status;
	uint8_t tries = 10;
	int ret;

#if DT_HAS_COMPAT_ON_BUS_STATUS_OKAY(st_lps27dewo, i3c)
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

	if (lps27dewo_id_get(ctx, &id) < 0) {
		LOG_ERR("%s: Not able to read dev id", dev->name);
		return -EIO;
	}

	if (id.whoami != LPS27DEWO_ID) {
		LOG_ERR("%s: Invalid chip ID 0x%02x", dev->name, id.whoami);
		return -EIO;
	}

	LOG_DBG("%s: chip id 0x%x", dev->name, id.whoami);

	/* Restore default configuration */
	if (lps27dewo_init_set(ctx, LPS27DEWO_RESET) < 0) {
		LOG_ERR("%s: Not able to reset device", dev->name);
		return -EIO;
	}

	do {
		if (!--tries) {
			LOG_DBG("sw reset timed out");
			return -ETIMEDOUT;
		}
		k_usleep(LPS2XDF_SWRESET_WAIT_TIME_US);

		if (lps27dewo_status_get(ctx, &status) < 0) {
			return -EIO;
		}
	} while (status.sw_reset);

	/* Set bdu and if_inc recommended for driver usage */
	if (lps27dewo_init_set(ctx, LPS27DEWO_DRV_RDY) < 0) {
		LOG_ERR("%s: Not able to set device to ready state", dev->name);
		return -EIO;
	}

	if (ON_I3C_BUS(cfg)) {
		lps27dewo_bus_mode_t bus_mode;

		/* Select bus interface */
		lps27dewo_bus_mode_get(ctx, &bus_mode);
		bus_mode.filter = LPS27DEWO_AUTO;
		bus_mode.interface = LPS27DEWO_SEL_BY_HW;
		lps27dewo_bus_mode_set(ctx, &bus_mode);
	}

	/* set sensor default odr */
	LOG_DBG("%s: odr: %d", dev->name, cfg->odr);
	ret = lps27dewo_mode_set_odr_raw(dev, cfg->odr);
	if (ret < 0) {
		LOG_ERR("%s: Failed to set odr %d", dev->name, cfg->odr);
		return ret;
	}

#ifdef CONFIG_LPS2XDF_TRIGGER
	if (cfg->trig_enabled) {
		if (lps2xdf_init_interrupt(dev, DEVICE_VARIANT_LPS27DEWO) < 0) {
			LOG_ERR("Failed to initialize interrupt.");
			return -EIO;
		}
	}
#endif

	return 0;
}
