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

#ifdef CONFIG_LIS2DW12_TRIGGER
static int lis2dw12_trig_cnt;
static int lis2dw12_trig_dil24_cnt;
static int lis2dw12_trig_tap_cnt;

static void lis2dw12_trigger_handler(const struct device *dev,
				     struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	lis2dw12_trig_cnt++;
}
static void lis2dw12_trigger_dil24_handler(const struct device *dev,
				     struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	lis2dw12_trig_dil24_cnt++;
}

#ifdef CONFIG_LIS2DW12_TAP
static void lis2dw12_trigger_tap_handler(const struct device *dev,
				     struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	lis2dw12_trig_tap_cnt++;
}
#endif

#endif

static void lis2dw12_config(const struct device *lis2dw12, uint16_t odr)
{
	struct sensor_value odr_attr, fs_attr;

	/* set LIS2DW12 accel/gyro sampling frequency to 100 Hz */
	odr_attr.val1 = odr;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lis2dw12, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LIS2DW12 accel\n");
		return;
	}

	sensor_g_to_ms2(16, &fs_attr);

	if (sensor_attr_set(lis2dw12, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set sampling frequency for LIS2DW12 gyro\n");
		return;
	}
}

void main(void)
{
	struct sensor_value accel[3], accel2[3];
	const struct device *lis2dw12 = device_get_binding("LIS2DW12");
	const struct device *lis2dw12_dil24 = device_get_binding("LIS2DW12_DIL24");
	int cnt = 1;
	int trig_ok = 0, trig_tap_ok = 0;
	int trig_dil24_ok = 0;

	if (lis2dw12 == NULL) {
		printf("Could not get LIS2DW12 device\n");
		return;
	}
	if (lis2dw12_dil24 == NULL) {
		printf("Could not get LIS2DW12_DIL24 device\n");
		return;
	}

	lis2dw12_config(lis2dw12, 200);
	lis2dw12_config(lis2dw12_dil24, 400);

#ifdef CONFIG_LIS2DW12_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	trig_ok = (sensor_trigger_set(lis2dw12, &trig, lis2dw12_trigger_handler) == 0) ?
		    1 : 0;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	trig_dil24_ok = (sensor_trigger_set(lis2dw12_dil24, &trig, lis2dw12_trigger_dil24_handler) == 0) ?
		    1 : 0;

#ifdef CONFIG_LIS2DW12_TAP
	trig.type = SENSOR_TRIG_TAP;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	trig_tap_ok = (sensor_trigger_set(lis2dw12_dil24, &trig, lis2dw12_trigger_tap_handler) == 0) ?
		    1 : 0;
#endif /* CONFIG_LIS2DW12_TAP */
#endif

	while (1) {
		/* Get sensor samples */

		if (!trig_ok && sensor_sample_fetch(lis2dw12) < 0) {
			printf("LIS2DW12 Sensor sample update error\n");
			return;
		}
		if (!trig_dil24_ok && sensor_sample_fetch(lis2dw12_dil24) < 0) {
			printf("LIS2DW12 DIL24 Sensor sample update error\n");
			return;
		}

		/* Get sensor data */

		sensor_channel_get(lis2dw12, SENSOR_CHAN_ACCEL_XYZ, accel);
		sensor_channel_get(lis2dw12_dil24, SENSOR_CHAN_ACCEL_XYZ, accel2);

		/* Display sensor data */

		/* Erase previous */
		printf("\0033\014");

		printf("X-NUCLEO-IKS01A3 sensor dashboard\n\n");

		printf("LIS2DW12: Accel (m.s-2): x: %.3f, y: %.3f, z: %.3f\n",
			sensor_value_to_double(&accel[0]),
			sensor_value_to_double(&accel[1]),
			sensor_value_to_double(&accel[2]));
		if (trig_ok)
			printk("%d:: lis2dw12 trig %d\n", cnt, lis2dw12_trig_cnt);

		printf("LIS2DW12 DIL24: Accel (m.s-2): x: %.3f, y: %.3f, z: %.3f\n",
			sensor_value_to_double(&accel2[0]),
			sensor_value_to_double(&accel2[1]),
			sensor_value_to_double(&accel2[2]));

		if (trig_dil24_ok)
			printk("%d:: lis2dw12 dil24 trig %d\n", cnt, lis2dw12_trig_dil24_cnt);

		if (trig_tap_ok)
			printk("%d:: lis2dw12 TAP trig %d\n", cnt, lis2dw12_trig_tap_cnt);

		cnt++;
		k_sleep(K_MSEC(2000));
	}
}
