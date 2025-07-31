/* ST Microelectronics IIS3DWB10IS accelerometer senor
 *
 * Copyright (c) 2025 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/iis3dwb10is.pdf
 */

#define DT_DRV_COMPAT st_iis3dwb10is

#include <zephyr/kernel.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>

#include "iis3dwb10is.h"

LOG_MODULE_DECLARE(IIS3DWB10IS, CONFIG_SENSOR_LOG_LEVEL);

/**
 * iis3dwb10is_route_int1 - enable selected int pin1 to generate interrupt
 */
int iis3dwb10is_route_int1(const struct device *dev, iis3dwb10is_pin_int_route_t pin_int)
{
	const struct iis3dwb10is_config *config = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&config->ctx;
	int ret;

	ret = iis3dwb10is_pin_int1_route_set(ctx, &pin_int);
	if (ret < 0) {
		LOG_ERR("%s: route on int1 error %d", dev->name, ret);
		return ret;
	}

	return 0;
}

/**
 * iis3dwb10is_route_int2 - enable selected int pin2 to generate interrupt
 */
int iis3dwb10is_route_int2(const struct device *dev, iis3dwb10is_pin_int_route_t pin_int)
{
	const struct iis3dwb10is_config *config = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&config->ctx;
	int ret;

	ret = iis3dwb10is_pin_int2_route_set(ctx, &pin_int);
	if (ret < 0) {
		LOG_ERR("%s: route on int2 error %d", dev->name, ret);
		return ret;
	}

	return 0;
}

static void iis3dwb10is_gpio_callback(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	struct iis3dwb10is_data *iis3dwb10is = CONTAINER_OF(cb, struct iis3dwb10is_data, gpio_cb);

	ARG_UNUSED(pins);

	gpio_pin_interrupt_configure_dt(iis3dwb10is->drdy_gpio, GPIO_INT_DISABLE);

	if (IS_ENABLED(CONFIG_IIS3DWB10IS_STREAM)) {
		iis3dwb10is_stream_irq_handler(iis3dwb10is->dev);
	}
}

int iis3dwb10is_init_interrupt(const struct device *dev)
{
	struct iis3dwb10is_data *iis3dwb10is = dev->data;
	const struct iis3dwb10is_config *cfg = dev->config;
	stmdev_ctx_t *ctx = (stmdev_ctx_t *)&cfg->ctx;
	int ret;

	iis3dwb10is->drdy_gpio = (cfg->drdy_pin == 1) ? (const struct gpio_dt_spec *)&cfg->int1_gpio
						  : (const struct gpio_dt_spec *)&cfg->int2_gpio;

	/* setup data ready gpio interrupt (INT1 or INT2) */
	if (!gpio_is_ready_dt(iis3dwb10is->drdy_gpio)) {
		LOG_ERR("Cannot get pointer to drdy_gpio device");
		return -ENODEV;
	}

	iis3dwb10is->dev = dev;

	ret = gpio_pin_configure_dt(iis3dwb10is->drdy_gpio, GPIO_INPUT);
	if (ret < 0) {
		LOG_ERR("Could not configure gpio");
		return ret;
	}

	gpio_init_callback(&iis3dwb10is->gpio_cb, iis3dwb10is_gpio_callback, BIT(iis3dwb10is->drdy_gpio->pin));

	if (gpio_add_callback(iis3dwb10is->drdy_gpio->port, &iis3dwb10is->gpio_cb) < 0) {
		LOG_DBG("Could not set gpio callback");
		return -EIO;
	}

	/* enable drdy on int1/int2 in pulse mode */
	iis3dwb10is_dataready_pulsed_t drdy =
		(cfg->drdy_pulsed) ? IIS3DWB10IS_DRDY_PULSED : IIS3DWB10IS_DRDY_LATCHED;
	if (iis3dwb10is_data_ready_mode_set(ctx, drdy)) {
		return -EIO;
	}

	return gpio_pin_interrupt_configure_dt(iis3dwb10is->drdy_gpio, GPIO_INT_EDGE_TO_ACTIVE);
}
