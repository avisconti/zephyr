/* ST Microelectronics IIS2DH 3-axis accelerometer driver
 *
 * Copyright (c) 2020 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Datasheet:
 * https://www.st.com/resource/en/datasheet/iis2dh.pdf
 */

#include <string.h>
#include <drivers/i2c.h>
#include <logging/log.h>

#include "iis2dh.h"

#ifdef DT_ST_IIS2DH_BUS_I2C

static u16_t iis2dh_i2c_slave_addr = DT_INST_0_ST_IIS2DH_BASE_ADDRESS;

LOG_MODULE_DECLARE(IIS2DH, CONFIG_SENSOR_LOG_LEVEL);

static int iis2dh_i2c_read(struct iis2dh_data *data, u8_t reg_addr,
			   u8_t *value, u16_t len)
{
	return i2c_burst_read(data->bus, iis2dh_i2c_slave_addr,
			      reg_addr | 0x80, value, len);
}

static int iis2dh_i2c_write(struct iis2dh_data *data, u8_t reg_addr,
			    u8_t *value, u16_t len)
{
	return i2c_burst_write(data->bus, iis2dh_i2c_slave_addr,
			       reg_addr | 0x80, value, len);
}

stmdev_ctx_t iis2dh_i2c_ctx = {
	.read_reg = (stmdev_read_ptr) iis2dh_i2c_read,
	.write_reg = (stmdev_write_ptr) iis2dh_i2c_write,
};

int iis2dh_i2c_init(struct device *dev)
{
	struct iis2dh_data *data = dev->driver_data;

	data->ctx = &iis2dh_i2c_ctx;
	data->ctx->handle = data;

	return 0;
}
#endif /* DT_ST_IIS2DH_BUS_I2C */
