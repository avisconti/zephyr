/*
 * Copyright (c) 2019 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <device.h>
#include <drivers/sensor.h>
#include <stdio.h>
#include <sys/util.h>

#ifdef CONFIG_LIS2MDL_TRIGGER
static int lis2mdl_trig_cnt;

static void lis2mdl_trigger_handler(const struct device *dev,
				    struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	lis2mdl_trig_cnt++;
}
#endif

static void lis2mdl_config(const struct device *lis2mdl, int odr,
			   sensor_trigger_handler_t handler)
{
	struct sensor_value odr_attr;

	/* set LIS2MDL sampling frequency to 100 Hz */
	odr_attr.val1 = odr;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lis2mdl, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LIS2MDL\n");
		return;
	}

#ifdef CONFIG_LIS2MDL_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_MAGN_XYZ;
	sensor_trigger_set(lis2mdl, &trig, handler);
#endif
}

static char *lis2mdl_label = "LIS2MDL";
static char *lis2mdl_dil24_label = "LIS2MDL_SPI";
void main(void)
{
	struct sensor_value die_temp2;
	struct sensor_value magn[3];
	const struct device *lis2mdl = device_get_binding(lis2mdl_label);
	const struct device *lis2mdl_dil24 = device_get_binding(lis2mdl_dil24_label);
	int cnt = 1;

	if (lis2mdl == NULL) {
		printf("Could not get LIS2MDL Magn device\n");
		return;
	}
	if (lis2mdl_dil24 == NULL) {
		printf("Could not get LIS2MDL DIL24 Magn device\n");
		return;
	}

	lis2mdl_config(lis2mdl, 100, NULL);

	while (1) {
		/* Get sensor samples */

#ifndef CONFIG_LIS2MDL_TRIGGER
		if (sensor_sample_fetch(lis2mdl) < 0) {
			printf("LIS2MDL Magn Sensor sample update error\n");
			return;
		}
#endif

		/* Get sensor data */
		sensor_channel_get(lis2mdl, SENSOR_CHAN_MAGN_XYZ, magn);
		sensor_channel_get(lis2mdl, SENSOR_CHAN_DIE_TEMP, &die_temp2);

		/* Display sensor data */

		/* Erase previous */
		printf("\0033\014");

		printf("X-NUCLEO-IKS01A3 sensor dashboard\n\n");

		/* lis2mdl */
		printf("LIS2MDL: Magn (gauss): x: %.3f, y: %.3f, z: %.3f\n",
		       sensor_value_to_double(&magn[0]),
		       sensor_value_to_double(&magn[1]),
		       sensor_value_to_double(&magn[2]));

		printf("LIS2MDL: Temperature: %.1f C\n",
		       sensor_value_to_double(&die_temp2));

#if defined(CONFIG_LIS2MDL_TRIGGER)
		printk("%d:: lis2mdl trig %d\n", cnt, lis2mdl_trig_cnt);
#endif

		cnt++;
		k_sleep(K_MSEC(2000));
	}
}
