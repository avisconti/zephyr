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

#ifdef CONFIG_IIS2MDC_TRIGGER
static int iis2mdc_trig_cnt;

static void iis2mdc_trigger_handler(const struct device *dev,
				    struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	iis2mdc_trig_cnt++;
}
#endif

#ifdef CONFIG_IIS2ICLX_TRIGGER
static int iis2iclx_acc_trig_cnt;

static void iis2iclx_acc_trigger_handler(const struct device *dev,
					   struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	iis2iclx_acc_trig_cnt++;
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

static void iis2mdc_config(const struct device *iis2mdc)
{
	struct sensor_value odr_attr;

	/* set IIS2MDC sampling frequency to 100 Hz */
	odr_attr.val1 = 100;
	odr_attr.val2 = 0;

	if (sensor_attr_set(iis2mdc, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for IIS2MDC\n");
		return;
	}

#ifdef CONFIG_IIS2MDC_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_MAGN_XYZ;
	sensor_trigger_set(iis2mdc, &trig, iis2mdc_trigger_handler);
#endif
}

static void iis2iclx_config(const struct device *iis2iclx)
{
	struct sensor_value odr_attr, fs_attr;

	/* set IIS2ICLX sampling frequency to 416 Hz */
	odr_attr.val1 = 416;
	odr_attr.val2 = 0;

	if (sensor_attr_set(iis2iclx, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for IIS2ICLX accel\n");
		return;
	}

	sensor_g_to_ms2(1000, &fs_attr);

	if (sensor_attr_set(iis2iclx, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set sampling frequency for IIS2ICLX accel\n");
		return;
	}

#ifdef CONFIG_IIS2ICLX_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(iis2iclx, &trig, iis2iclx_acc_trigger_handler);

#endif
}

void main(void)
{
#ifdef CONFIG_IIS2ICLX_ENABLE_TEMP
	struct sensor_value die_temp;
#endif
	struct sensor_value die_temp2;
	struct sensor_value accel1[3], accel2[3];
	struct sensor_value gyro[3];
	struct sensor_value magn[3];
	const struct device *iis2dlpc = device_get_binding(DT_LABEL(DT_INST(0, st_iis2dlpc)));
	const struct device *iis2mdc = device_get_binding(DT_LABEL(DT_INST(0, st_iis2mdc)));
	const struct device *iis2iclx = device_get_binding(DT_LABEL(DT_INST(0, st_iis2iclx)));
	int cnt = 1;

	if (iis2dlpc == NULL) {
		printf("Could not get IIS2DLPC device\n");
		return;
	}
	if (iis2mdc == NULL) {
		printf("Could not get IIS2MDC Magn device\n");
		return;
	}
	if (iis2iclx == NULL) {
		printf("Could not get IIS2ICLX IMU device\n");
		return;
	}

	iis2dlpc_config(iis2dlpc);
	iis2mdc_config(iis2mdc);
	iis2iclx_config(iis2iclx);

	while (1) {
		/* Get sensor samples */

#ifndef CONFIG_IIS2DLPC_TRIGGER
		if (sensor_sample_fetch(iis2dlpc) < 0) {
			printf("IIS2DLPC Sensor sample update error\n");
			return;
		}
#endif
#ifndef CONFIG_IIS2MDC_TRIGGER
		if (sensor_sample_fetch(iis2mdc) < 0) {
			printf("IIS2MDC Magn Sensor sample update error\n");
			return;
		}
#endif
#ifndef CONFIG_IIS2ICLX_TRIGGER
		if (sensor_sample_fetch(iis2iclx) < 0) {
			printf("IIS2ICLX IMU Sensor sample update error\n");
			return;
		}
#endif

		/* Get sensor data */

		sensor_channel_get(iis2dlpc, SENSOR_CHAN_ACCEL_XYZ, accel2);
		sensor_channel_get(iis2mdc, SENSOR_CHAN_MAGN_XYZ, magn);
		sensor_channel_get(iis2mdc, SENSOR_CHAN_DIE_TEMP, &die_temp2);
		sensor_channel_get(iis2iclx, SENSOR_CHAN_ACCEL_XYZ, accel1);

		/* Display sensor data */

		/* Erase previous */
		printf("\0033\014");

		printf("X-NUCLEO-IKS02A1 sensor Mode 1 dashboard\n\n");

		printf("IIS2DLPC: Accel (m.s-2): x: %.3f, y: %.3f, z: %.3f\n",
			sensor_value_to_double(&accel2[0]),
			sensor_value_to_double(&accel2[1]),
			sensor_value_to_double(&accel2[2]));

		/* iis2mdc */
		printf("IIS2MDC: Magn (gauss): x: %.3f, y: %.3f, z: %.3f\n",
		       sensor_value_to_double(&magn[0]),
		       sensor_value_to_double(&magn[1]),
		       sensor_value_to_double(&magn[2]));

		printf("IIS2MDC: Temperature: %.1f C\n",
		       sensor_value_to_double(&die_temp2));

		printf("IIS2ICLX: Accel (m.s-2): x: %.3f, y: %.3f, \n",
			sensor_value_to_double(&accel1[0]),
			sensor_value_to_double(&accel1[1]));

#ifdef CONFIG_IIS2DLPC_TRIGGER
		printk("%d:: iis2dlpc trig %d\n", cnt, iis2dlpc_trig_cnt);
#endif

#if defined(CONFIG_IIS2MDC_TRIGGER)
		printk("%d:: iis2mdc trig %d\n", cnt, iis2mdc_trig_cnt);
#endif

#ifdef CONFIG_IIS2ICLX_TRIGGER
		printk("%d:: iis2iclx acc trig %d\n", cnt, iis2iclx_acc_trig_cnt);
#endif


		cnt++;
		k_sleep(K_MSEC(2000));
	}
}
