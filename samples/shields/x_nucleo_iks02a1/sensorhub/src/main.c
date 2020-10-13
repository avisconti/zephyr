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
#endif

#ifdef CONFIG_IIS2ICLX_TRIGGER
static int iis2iclx_0_acc_trig_cnt;
static int iis2iclx_1_acc_trig_cnt;
static int iis2iclx_0_temp_trig_cnt;

static void iis2iclx_acc_0_trig_handler(const struct device *dev,
					struct sensor_trigger *trig)
{
	sensor_sample_fetch(dev);
	iis2iclx_0_acc_trig_cnt++;
}

static void iis2iclx_acc_1_trig_handler(const struct device *dev,
					struct sensor_trigger *trig)
{
	sensor_sample_fetch(dev);
	iis2iclx_1_acc_trig_cnt++;
}


static void iis2iclx_temp_trig_handler(const struct device *dev,
					 struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_DIE_TEMP);
	iis2iclx_0_temp_trig_cnt++;
}
#endif

static void iis2dlpc_config(const struct device *iis2dlpc)
{
	struct sensor_value odr_attr, fs_attr;

	/* set IIS2DLPC accel/gyro sampling frequency to 100 Hz */
	odr_attr.val1 = 100;
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
	sensor_trigger_set(iis2dlpc, &trig, iis2dlpc_trigger_handler);
#endif
}

static void iis2iclx_config(const struct device *iis2iclx, int odr)
{
	struct sensor_value odr_attr, fs_attr;

	/* set IIS2ICLX accel sampling frequency to 208 Hz */
	odr_attr.val1 = odr;
	odr_attr.val2 = 0;

	if (sensor_attr_set(iis2iclx, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for IIS2ICLX accel\n");
		return;
	}

	sensor_g_to_ms2(1000, &fs_attr);

	if (sensor_attr_set(iis2iclx, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set fs for IIS2ICLX accel\n");
		return;
	}

	/* set IIS2ICLX external magn sampling frequency to 100 Hz */
	odr_attr.val1 = 100;
	odr_attr.val2 = 0;

#ifdef CONFIG_IIS2ICLX_EXT_IIS2MDC
	if (sensor_attr_set(iis2iclx, SENSOR_CHAN_MAGN_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for IIS2ICLX ext magn\n");
		return;
	}
#endif
}

static const char *iis2dlpc_label = "IIS2DLPC";		/* on shield */
static const char *iis2iclx_0_label = "IIS2ICLX_0";	/* on shield */
static const char *iis2iclx_1_label = "IIS2ICLX_1";	/* on DIL24 */

void main(void)
{
#ifdef CONFIG_IIS2ICLX_ENABLE_TEMP
	struct sensor_value die_temp;
#endif
	struct sensor_value accel1[2], accel2[3], accel3[2];
	struct sensor_value magn[3];
	const struct device *iis2dlpc = device_get_binding(iis2dlpc_label);
	const struct device *iis2iclx_0 = device_get_binding(iis2iclx_0_label);
	const struct device *iis2iclx_1 = device_get_binding(iis2iclx_1_label);
	bool iis2iclx_0_trigger = false, iis2iclx_1_trigger = false;
	int cnt = 1;

	if (iis2dlpc == NULL) {
		printf("Could not get IIS2DLPC device\n");
	} else {
		iis2dlpc_config(iis2dlpc);
	}
	if (iis2iclx_0 == NULL) {
		printf("Could not get IIS2ICLX 0 device\n");
	} else {
		iis2iclx_config(iis2iclx_0, 208);
	}
	if (iis2iclx_1 == NULL) {
		printf("Could not get IIS2ICLX 1 device\n");
	} else {
		iis2iclx_config(iis2iclx_1, 416);
	}

#ifdef CONFIG_IIS2ICLX_TRIGGER
	struct sensor_trigger trig;

	if (iis2iclx_0) {
		trig.type = SENSOR_TRIG_DATA_READY;
		trig.chan = SENSOR_CHAN_ACCEL_XYZ;
		if (sensor_trigger_set(iis2iclx_0, &trig, iis2iclx_acc_0_trig_handler) >=0)
			iis2iclx_0_trigger = true;

		trig.type = SENSOR_TRIG_DATA_READY;
		trig.chan = SENSOR_CHAN_DIE_TEMP;
		sensor_trigger_set(iis2iclx_0, &trig, iis2iclx_temp_trig_handler);
	}

	if (iis2iclx_1) {
		trig.type = SENSOR_TRIG_DATA_READY;
		trig.chan = SENSOR_CHAN_ACCEL_XYZ;
		if (sensor_trigger_set(iis2iclx_1, &trig, iis2iclx_acc_1_trig_handler) >= 0)
			iis2iclx_1_trigger = true;
	}

#endif

	while (1) {
		/* Get sensor samples */

#ifndef CONFIG_IIS2DLPC_TRIGGER
		if (iis2dlpc && sensor_sample_fetch(iis2dlpc) < 0) {
			printf("IIS2DLPC Sensor sample update error\n");
			return;
		}
#endif
		if (iis2iclx_0 && !iis2iclx_0_trigger && sensor_sample_fetch(iis2iclx_0) < 0) {
			printf("IIS2ICLX 0 Sensor sample update error\n");
			return;
		}

		if (iis2iclx_1 && !iis2iclx_1_trigger && sensor_sample_fetch(iis2iclx_1) < 0) {
			printf("IIS2ICLX 1 Sensor sample update error\n");
			return;
		}

		/* Get sensor data */
		if (iis2dlpc) {
			sensor_channel_get(iis2dlpc, SENSOR_CHAN_ACCEL_XYZ, accel2);
		}

		if (iis2iclx_0) {
			sensor_channel_get(iis2iclx_0, SENSOR_CHAN_ACCEL_XYZ, accel1);
#ifdef CONFIG_IIS2ICLX_ENABLE_TEMP
			sensor_channel_get(iis2iclx_0, SENSOR_CHAN_DIE_TEMP, &die_temp);
#endif
#ifdef CONFIG_IIS2ICLX_EXT_IIS2MDC
			sensor_channel_get(iis2iclx_0, SENSOR_CHAN_MAGN_XYZ, magn);
#endif
		}

		if (iis2iclx_1) {
			sensor_channel_get(iis2iclx_1, SENSOR_CHAN_ACCEL_XYZ, accel3);
		}

		/* Display sensor data */

		/* Erase previous */
		printf("\0033\014");

		printf("X-NUCLEO-IKS02A1 sensor Mode 2 dashboard\n\n");

		if (iis2dlpc) {
			printf("IIS2DLPC: Accel (m.s-2): x: %.3f, y: %.3f, z: %.3f\n",
				sensor_value_to_double(&accel2[0]),
				sensor_value_to_double(&accel2[1]),
				sensor_value_to_double(&accel2[2]));
		}

		if (iis2iclx_0) {
			printf("IIS2ICLX: Accel 0 (m.s-2): x: %.3f, y: %.3f, \n",
				sensor_value_to_double(&accel1[0]),
				sensor_value_to_double(&accel1[1]));

#ifdef CONFIG_IIS2ICLX_ENABLE_TEMP
			/* temperature */
			printf("IIS2ICLX: Temperature: %.1f C\n",
			    sensor_value_to_double(&die_temp));
#endif

#ifdef CONFIG_IIS2ICLX_EXT_IIS2MDC
			printf("IIS2ICLX: Magn (gauss): x: %.3f, y: %.3f, z: %.3f\n",
			    sensor_value_to_double(&magn[0]),
			    sensor_value_to_double(&magn[1]),
			    sensor_value_to_double(&magn[2]));
#endif
		}

		if (iis2iclx_1) {
			printf("IIS2ICLX: Accel 1 (m.s-2): x: %.3f, y: %.3f, \n",
				sensor_value_to_double(&accel3[0]),
				sensor_value_to_double(&accel3[1]));
		}
#ifdef CONFIG_IIS2DLPC_TRIGGER
		printk("%d:: iis2dlpc trig %d\n", cnt, iis2dlpc_trig_cnt);
#endif

#ifdef CONFIG_IIS2ICLX_TRIGGER
		if (iis2iclx_0 && iis2iclx_0_trigger) {
			printk("%d:: iis2iclx acc 0 trig %d\n", cnt, iis2iclx_0_acc_trig_cnt);
#ifdef CONFIG_IIS2ICLX_ENABLE_TEMP
			printk("%d:: iis2iclx temp trig %d\n", cnt, iis2iclx_0_temp_trig_cnt);
#endif
		}
		if (iis2iclx_1 && iis2iclx_1_trigger) {
			printk("%d:: iis2iclx acc 1 trig %d\n", cnt, iis2iclx_1_acc_trig_cnt);
		}
#endif

		cnt++;
		k_sleep(K_MSEC(2000));
	}
}
