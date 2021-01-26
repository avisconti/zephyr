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

#ifdef CONFIG_IIS2MDC_TRIGGER
static int iis2mdc_trig_cnt;
static int iis2mdc_trig_dil24_cnt;

static void iis2mdc_trigger_handler(const struct device *dev,
				    struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	iis2mdc_trig_cnt++;
}

static void iis2mdc_trigger_handler_dil24(const struct device *dev,
				    struct sensor_trigger *trig)
{
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ALL);
	iis2mdc_trig_dil24_cnt++;
}
#endif

static void iis2mdc_config(const struct device *iis2mdc, uint16_t odr)
{
	struct sensor_value odr_attr;

	/* set IIS2MDC sampling frequency to 100 Hz */
	odr_attr.val1 = odr;
	odr_attr.val2 = 0;

	if (sensor_attr_set(iis2mdc, SENSOR_CHAN_ALL,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &odr_attr) < 0) {
		printk("Cannot set sampling frequency for IIS2MDC\n");
		return;
	}
}

static const char *iis2mdc_shield_label = "IIS2MDC";              /* on shield */
static const char *iis2mdc_dil24_label = "IIS2MDC_SPI";           /* on DIL24 */

void main(void)
{
	struct sensor_value die_temp2;
	struct sensor_value magn[3];
	const struct device *iis2mdc = device_get_binding(iis2mdc_shield_label);
	//const struct device *iis2mdc_dil24 = device_get_binding(iis2mdc_dil24_label);
	int cnt = 1;

	if (iis2mdc == NULL) {
		printf("Could not get IIS2MDC Magn device\n");
		return;
	}
	//if (iis2mdc_dil24 == NULL) {
		//printf("Could not get DIL24 IIS2MDC Magn device\n");
		//return;
	//}

	iis2mdc_config(iis2mdc, 100);
	//iis2mdc_config(iis2mdc_dil24, 200);

#ifdef CONFIG_IIS2MDC_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_MAGN_XYZ;
	sensor_trigger_set(iis2mdc, &trig, iis2mdc_trigger_handler);
	//sensor_trigger_set(iis2mdc_dil24, &trig, iis2mdc_trigger_handler_dil24);
#endif

	while (1) {
		/* Get sensor samples */

#ifndef CONFIG_IIS2MDC_TRIGGER
		if (sensor_sample_fetch(iis2mdc) < 0) {
			printf("IIS2MDC Magn Sensor sample update error\n");
			return;
		}
#endif

		/* Get sensor data */

		sensor_channel_get(iis2mdc, SENSOR_CHAN_MAGN_XYZ, magn);
		sensor_channel_get(iis2mdc, SENSOR_CHAN_DIE_TEMP, &die_temp2);
		//sensor_channel_get(iis2mdc_dil24, SENSOR_CHAN_MAGN_XYZ, magn);
		//sensor_channel_get(iis2mdc_dil24, SENSOR_CHAN_DIE_TEMP, &die_temp2);

		/* Display sensor data */

		/* Erase previous */
		printf("\0033\014");

		printf("X-NUCLEO-IKS02A1 sensor Mode 1 dashboard\n\n");

		/* iis2mdc */
		printf("IIS2MDC: Magn (gauss): x: %.3f, y: %.3f, z: %.3f\n",
		       sensor_value_to_double(&magn[0]),
		       sensor_value_to_double(&magn[1]),
		       sensor_value_to_double(&magn[2]));

		printf("IIS2MDC: Temperature: %.1f C\n",
		       sensor_value_to_double(&die_temp2));

#if defined(CONFIG_IIS2MDC_TRIGGER)
		printk("%d:: iis2mdc trig %d\n", cnt, iis2mdc_trig_cnt);
#endif

		cnt++;
		k_sleep(K_MSEC(2000));
	}
}
