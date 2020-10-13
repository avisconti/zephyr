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

#ifdef CONFIG_IIS2ICLX_TRIGGER
static int iis2iclx_i2c_trig_cnt;
static int iis2iclx_spi_trig_cnt;

static void iis2iclx_i2c_trigger_handler(const struct device *dev,
					   struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	iis2iclx_i2c_trig_cnt++;
}

static void iis2iclx_spi_trigger_handler(const struct device *dev,
					   struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	iis2iclx_spi_trig_cnt++;
}

#endif

static void iis2iclx_config(const struct device *iis2iclx, uint16_t odr)
{
	struct sensor_value odr_attr, fs_attr;

	/* set IIS2ICLX sampling frequency to 416 Hz */
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
		printk("Cannot set sampling frequency for IIS2ICLX accel\n");
		return;
	}
}

void main(void)
{
#ifdef CONFIG_IIS2ICLX_ENABLE_TEMP
	struct sensor_value die_temp;
#endif
	struct sensor_value accel1[3];
	struct sensor_value accel2[3];
	const struct device *iis2iclx_i2c = device_get_binding("IIS2ICLX_I2C");
	const struct device *iis2iclx_spi = device_get_binding("IIS2ICLX_SPI");
	int cnt = 1;

	if (iis2iclx_i2c == NULL) {
		printf("Could not get IIS2ICLX I2C device\n");
		return;
	}

	if (iis2iclx_spi == NULL) {
		printf("Could not get IIS2ICLX SPI device\n");
		return;
	}

	iis2iclx_config(iis2iclx_i2c, 208);
	iis2iclx_config(iis2iclx_spi, 416);

#ifdef CONFIG_IIS2ICLX_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(iis2iclx_i2c, &trig, iis2iclx_i2c_trigger_handler);
	sensor_trigger_set(iis2iclx_spi, &trig, iis2iclx_spi_trigger_handler);

#endif

	while (1) {
		/* Get sensor samples */

#ifndef CONFIG_IIS2ICLX_TRIGGER
		if (sensor_sample_fetch(iis2iclx_i2c) < 0) {
			printf("IIS2ICLX i2c Sensor sample update error\n");
			return;
		}
		if (sensor_sample_fetch(iis2iclx_spi) < 0) {
			printf("IIS2ICLX spi Sensor sample update error\n");
			return;
		}
#endif

		/* Get sensor data */

		sensor_channel_get(iis2iclx_i2c, SENSOR_CHAN_ACCEL_XYZ, accel1);
		sensor_channel_get(iis2iclx_spi, SENSOR_CHAN_ACCEL_XYZ, accel2);

		/* Display sensor data */

		/* Erase previous */
		printf("\0033\014");

		printf("X-NUCLEO-IKS02A1 sensor Mode 1 dashboard\n\n");

		printf("IIS2ICLX i2c: Accel (m.s-2): x: %.3f, y: %.3f, \n",
			sensor_value_to_double(&accel1[0]),
			sensor_value_to_double(&accel1[1]));
		printf("IIS2ICLX spi: Accel (m.s-2): x: %.3f, y: %.3f, \n",
			sensor_value_to_double(&accel2[0]),
			sensor_value_to_double(&accel2[1]));


#ifdef CONFIG_IIS2ICLX_TRIGGER
		printk("%d:: iis2iclx i2c trig %d\n", cnt, iis2iclx_i2c_trig_cnt);
		printk("%d:: iis2iclx spi trig %d\n", cnt, iis2iclx_spi_trig_cnt);
#endif

		cnt++;
		k_sleep(K_MSEC(2000));
	}
}
