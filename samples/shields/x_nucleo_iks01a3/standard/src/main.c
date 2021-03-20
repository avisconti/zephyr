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

#ifdef CONFIG_LSM6DSO_TRIGGER
static int lsm6dso_acc_trig_dil24_cnt;
static int lsm6dso_gyr_trig_dil24_cnt;
static int lsm6dso_acc_trig_cnt;
static int lsm6dso_gyr_trig_cnt;

static void lsm6dso_acc_trig_dil24_handler(const struct device *dev,
				     struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	lsm6dso_acc_trig_dil24_cnt++;
}

static void lsm6dso_gyr_trig_dil24_handler(const struct device *dev,
				     struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	lsm6dso_gyr_trig_dil24_cnt++;
}

static void lsm6dso_acc_trig_handler(const struct device *dev,
				     struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	lsm6dso_acc_trig_cnt++;
}

static void lsm6dso_gyr_trig_handler(const struct device *dev,
				     struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	lsm6dso_gyr_trig_cnt++;
}
#endif

static void lsm6dso_config(const struct device *lsm6dso, uint16_t odr)
{
	struct sensor_value odr_attr, fs_attr;

	/* set LSM6DSO accel sampling frequency to odr Hz */
	odr_attr.val1 = odr;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dso, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LSM6DSO accel\n");
		return;
	}

	sensor_g_to_ms2(16, &fs_attr);

	if (sensor_attr_set(lsm6dso, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set fs for LSM6DSO accel\n");
		return;
	}

	/* set LSM6DSO gyro sampling frequency to odr Hz */
	odr_attr.val1 = odr;
	odr_attr.val2 = 0;

	if (sensor_attr_set(lsm6dso, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for LSM6DSO gyro\n");
		return;
	}

	sensor_degrees_to_rad(250, &fs_attr);

	if (sensor_attr_set(lsm6dso, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &fs_attr) < 0) {
		printk("Cannot set fs for LSM6DSO gyro\n");
		return;
	}

}

static const char *lsm6dso_shield = "LSM6DSO";
static const char *lsm6dso_dil24_label = "LSM6DSO_SPI";
void main(void)
{
	struct sensor_value accel1[3], accel2[3];
	struct sensor_value gyro[3], gyro2[3];
	const struct device *lsm6dso = device_get_binding(lsm6dso_shield);
	const struct device *lsm6dso_dil24 = device_get_binding(lsm6dso_dil24_label);
	int cnt = 1;

	if (lsm6dso == NULL) {
		printf("Could not get LSM6DSO device\n");
		return;
	}
	if (lsm6dso_dil24 == NULL) {
		printf("Could not get LSM6DSO DIL24 device\n");
		return;
	}

	lsm6dso_config(lsm6dso, 208);
	lsm6dso_config(lsm6dso_dil24, 416);

#ifdef CONFIG_LSM6DSO_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(lsm6dso_dil24, &trig, lsm6dso_acc_trig_dil24_handler);

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_GYRO_XYZ;
	sensor_trigger_set(lsm6dso_dil24, &trig, lsm6dso_gyr_trig_dil24_handler);

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(lsm6dso, &trig, lsm6dso_acc_trig_handler);

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_GYRO_XYZ;
	sensor_trigger_set(lsm6dso, &trig, lsm6dso_gyr_trig_handler);
#endif

	while (1) {
		/* Get sensor samples */

#ifndef CONFIG_LSM6DSO_TRIGGER
		if (sensor_sample_fetch(lsm6dso) < 0) {
			printf("LSM6DSO Sensor sample update error\n");
			return;
		}
		if (sensor_sample_fetch(lsm6dso_dil24) < 0) {
			printf("LSM6DSO DIL24 Sensor sample update error\n");
			return;
		}
#endif

		/* Get sensor data */

		sensor_channel_get(lsm6dso_dil24, SENSOR_CHAN_ACCEL_XYZ, accel2);
		sensor_channel_get(lsm6dso_dil24, SENSOR_CHAN_GYRO_XYZ, gyro2);

		sensor_channel_get(lsm6dso, SENSOR_CHAN_ACCEL_XYZ, accel1);
		sensor_channel_get(lsm6dso, SENSOR_CHAN_GYRO_XYZ, gyro);

		/* Display sensor data */

		/* Erase previous */
		printf("\0033\014");

		printf("X-NUCLEO-IKS01A3 sensor dashboard\n\n");

		printf("LSM6DSO: DIL24 Accel (m.s-2): x: %.3f, y: %.3f, z: %.3f\n",
			sensor_value_to_double(&accel2[0]),
			sensor_value_to_double(&accel2[1]),
			sensor_value_to_double(&accel2[2]));
		printf("LSM6DSO: DIL24 GYro (dps): x: %.3f, y: %.3f, z: %.3f\n",
			sensor_value_to_double(&gyro2[0]),
			sensor_value_to_double(&gyro2[1]),
			sensor_value_to_double(&gyro2[2]));


		printf("LSM6DSO: Accel (m.s-2): x: %.3f, y: %.3f, z: %.3f\n",
			sensor_value_to_double(&accel1[0]),
			sensor_value_to_double(&accel1[1]),
			sensor_value_to_double(&accel1[2]));
		printf("LSM6DSO: GYro (dps): x: %.3f, y: %.3f, z: %.3f\n",
			sensor_value_to_double(&gyro[0]),
			sensor_value_to_double(&gyro[1]),
			sensor_value_to_double(&gyro[2]));


#ifdef CONFIG_LSM6DSO_TRIGGER
		printk("%d:: lsm6dso acc DIL24 trig %d\n", cnt, lsm6dso_acc_trig_dil24_cnt);
		printk("%d:: lsm6dso acc DIL24 trig %d\n", cnt, lsm6dso_gyr_trig_dil24_cnt);
		printk("%d:: lsm6dso acc trig %d\n", cnt, lsm6dso_acc_trig_cnt);
		printk("%d:: lsm6dso gyr trig %d\n", cnt, lsm6dso_gyr_trig_cnt);
#endif

		cnt++;
		k_sleep(K_MSEC(2000));
	}
}
