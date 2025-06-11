/* ST Microelectronics LPS27DEWO pressure and temperature sensor
 *
 * Copyright (c) 2025 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>
#include <stmemsc.h>

#include <zephyr/drivers/sensor.h>

#include "lps27dewo_reg.h"

#ifndef ZEPHYR_DRIVERS_SENSOR_LPS27DEWO_LPS27DEWO_H_
#define ZEPHYR_DRIVERS_SENSOR_LPS27DEWO_LPS27DEWO_H_

extern const struct lps2xdf_chip_api st_lps27dewo_chip_api;

int st_lps27dewo_init(const struct device *dev);

#endif /* ZEPHYR_DRIVERS_SENSOR_LPS27DEWO_H_ */
