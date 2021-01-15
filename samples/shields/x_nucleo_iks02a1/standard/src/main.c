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

#ifdef CONFIG_IIS2DLPC_TRIGGER
static int iis2dlpc_trig_cnt;

static void iis2dlpc_trigger_handler(const struct device *dev,
				     struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	iis2dlpc_trig_cnt++;
}

static int iis2dlpc_trig_cnt2;

static void iis2dlpc_trigger_handler2(const struct device *dev,
				     struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	iis2dlpc_trig_cnt2++;
}
#endif

static void iis2dlpc_config(const struct device *iis2dlpc, uint16_t odr, void (hand()))
{
	struct sensor_value odr_attr, fs_attr;

	/* set IIS2DLPC accel/gyro sampling frequency to 100 Hz */
	odr_attr.val1 = odr;
	odr_attr.val2 = 0;

	if (sensor_attr_set(iis2dlpc, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for IIS2DLPC accel\n");
		return;
	}

	sensor_g_to_ms2(16, &fs_attr);

	if (sensor_attr_set(iis2dlpc, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set sampling frequency for IIS2DLPC gyro\n");
		return;
	}

#ifdef CONFIG_IIS2DLPC_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(iis2dlpc, &trig, hand);
#endif
}

static const char *iis2dlpc_shield_label = "IIS2DLPC";              /* on shield */
static const char *iis2dlpc_dil24_label = "IIS2DLPC_SPI";           /* on DIL24 */

void main(void)
{
#ifdef CONFIG_ISM330DHCX_ENABLE_TEMP
	struct sensor_value die_temp;
#endif
	struct sensor_value accel2[3], accel3[3];
	const struct device *iis2dlpc = device_get_binding(iis2dlpc_shield_label);
	const struct device *iis2dlpc_dil24 = device_get_binding(iis2dlpc_dil24_label);
	int cnt = 1;

	if (iis2dlpc == NULL) {
		printf("Could not get IIS2DLPC device\n");
		return;
	}
	if (iis2dlpc_dil24 == NULL) {
		printf("Could not get IIS2DLPC DIL24 device\n");
		return;
	}

	iis2dlpc_config(iis2dlpc, 100, iis2dlpc_trigger_handler);
	iis2dlpc_config(iis2dlpc_dil24, 400, iis2dlpc_trigger_handler2);

	while (1) {
		/* Get sensor samples */

#ifndef CONFIG_IIS2DLPC_TRIGGER
		if (sensor_sample_fetch(iis2dlpc) < 0) {
			printf("IIS2DLPC Sensor sample update error\n");
			return;
		}
		if (sensor_sample_fetch(iis2dlpc_dil24) < 0) {
			printf("IIS2DLPC2 Sensor sample update error\n");
			return;
		}
#endif
		/* Get sensor data */

		sensor_channel_get(iis2dlpc, SENSOR_CHAN_ACCEL_XYZ, accel2);
		sensor_channel_get(iis2dlpc_dil24, SENSOR_CHAN_ACCEL_XYZ, accel3);

		/* Display sensor data */

		/* Erase previous */
		printf("\0033\014");

		printf("X-NUCLEO-IKS02A1 sensor Mode 1 dashboard\n\n");

		printf("IIS2DLPC: Accel (m.s-2): x: %.3f, y: %.3f, z: %.3f\n",
			sensor_value_to_double(&accel2[0]),
			sensor_value_to_double(&accel2[1]),
			sensor_value_to_double(&accel2[2]));

		printf("IIS2DLPC2: Accel (m.s-2): x: %.3f, y: %.3f, z: %.3f\n",
			sensor_value_to_double(&accel3[0]),
			sensor_value_to_double(&accel3[1]),
			sensor_value_to_double(&accel3[2]));

#ifdef CONFIG_IIS2DLPC_TRIGGER
		printk("%d:: iis2dlpc trig %d\n", cnt, iis2dlpc_trig_cnt);
		printk("%d:: iis2dlpc2 trig %d\n", cnt, iis2dlpc_trig_cnt2);
#endif

		cnt++;
		k_sleep(K_MSEC(2000));
	}
}
