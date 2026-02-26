/*
 * Copyright (c) 2025 Croxel Inc.
 * Copyright (c) 2025 CogniPilot Foundation
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Header file for extended sensor API of AFBR-S50 sensor
 * @ingroup afbr_s50_interface
 */

#ifndef ZEPHYR_DRIVERS_SENSOR_LSM6DSV16X_H_
#define ZEPHYR_DRIVERS_SENSOR_LSM6DSV16X_H_

/**
 * @brief Broadcom AFBR-S50 3D ToF sensor
 * @defgroup afbr_s50_interface AFBR-S50
 * @ingroup sensor_interface_ext
 * @{
 */

#include <zephyr/drivers/sensor.h>

#ifdef __cplusplus
extern "C" {
#endif


/* LSM6DSV16X specific channels */
enum sensor_channel_lsm6dsv16x {
	/** Step counter */
	SENSOR_CHAN_LSM6DSV16X_STEP_COUNTER = SENSOR_CHAN_PRIV_START,
	/** MLC Result */
	SENSOR_CHAN_LSM6DSV16X_MLC_RESULT,
};

/**
 * @brief Sensor trigger types.
 */
enum sensor_trigger_type_lsm6dsv16x {
      /** Step counter */
      SENSOR_TRIG_LSM6DSV16X_STEP_DETECTION = SENSOR_TRIG_PRIV_START,
};


struct lsm6dsv16x_sensor_step_counter_data {
	struct sensor_data_header header;
	int8_t shift;
	struct lsm6dsv16x_sensor_step_counter_sample_data {
		uint32_t timestamp;
		uint16_t counter;
	} readings[1];
};

#ifdef __cplusplus
}
#endif

/**
 * @}
 */

#endif /* ZEPHYR_DRIVERS_SENSOR_LSM6DSV16X_H_ */
