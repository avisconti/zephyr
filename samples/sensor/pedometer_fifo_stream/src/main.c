/*
 * Copyright (c) 2024 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <stdlib.h>

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/kernel.h>
#include <zephyr/rtio/rtio.h>
#include <zephyr/drivers/sensor/lsm6dsv16x.h>

#define STREAMDEV_ALIAS(i) DT_ALIAS(_CONCAT(stream, i))
#define STREAMDEV_DEVICE(i, _) \
	IF_ENABLED(DT_NODE_EXISTS(STREAMDEV_ALIAS(i)), (DEVICE_DT_GET(STREAMDEV_ALIAS(i)),))
#define NUM_SENSORS 1

/* support up to 10 sensors */
static const struct device *const sensors[] = { LISTIFY(10, STREAMDEV_DEVICE, ()) };

#define STREAM_IODEV_SYM(id) CONCAT(stream_iodev, id)
#define STREAM_IODEV_PTR(id, _) &STREAM_IODEV_SYM(id)

#define STREAM_TRIGGERS					   \
	{ SENSOR_TRIG_LSM6DSV16X_STEP_DETECTION, SENSOR_STREAM_DATA_INCLUDE }, \
	{ SENSOR_TRIG_FIFO_FULL, SENSOR_STREAM_DATA_NOP }, \
	{ SENSOR_TRIG_FIFO_WATERMARK, SENSOR_STREAM_DATA_INCLUDE }

#define STREAM_DEFINE_IODEV(id, _)    \
	SENSOR_DT_STREAM_IODEV(	      \
		STREAM_IODEV_SYM(id), \
		STREAMDEV_ALIAS(id),  \
		STREAM_TRIGGERS);

LISTIFY(NUM_SENSORS, STREAM_DEFINE_IODEV, (;));

struct rtio_iodev *iodevs[NUM_SENSORS] = { LISTIFY(NUM_SENSORS, STREAM_IODEV_PTR, (,)) };

RTIO_DEFINE_WITH_MEMPOOL(stream_ctx, NUM_SENSORS, NUM_SENSORS,
			 NUM_SENSORS * 20, 256, sizeof(void *));

struct sensor_chan_spec accel_chan = { SENSOR_CHAN_ACCEL_XYZ, 0 };
struct sensor_chan_spec pedo_chan = { SENSOR_CHAN_LSM6DSV16X_STEP_COUNTER, 0 };

#define TASK_STACK_SIZE           2048ul
static K_THREAD_STACK_ARRAY_DEFINE(thread_stack, NUM_SENSORS, TASK_STACK_SIZE);
static struct k_thread thread_id[NUM_SENSORS];

static void print_stream(void *p1, void *p2, void *p3)
{
	const struct device *dev = (const struct device *)p1;
	struct rtio_iodev *iodev = (struct rtio_iodev *)p2;
	int rc = 0;
	const struct sensor_decoder_api *decoder;
	struct rtio_cqe *cqe;
	uint8_t *buf;
	uint32_t buf_len;
	struct rtio_sqe *handle;
	uint8_t accel_buf[128] = { 0 };
	uint8_t pedo_buf[128] = { 0 };
	struct sensor_three_axis_data *accel_data = (struct sensor_three_axis_data *)accel_buf;
	struct sensor_q31_data *pedo_data = (struct sensor_q31_data *)pedo_buf;

	/* Start the stream */
	printk("sensor_stream\n");
	sensor_stream(iodev, &stream_ctx, NULL, &handle);

	while (1) {
		cqe = rtio_cqe_consume_block(&stream_ctx);

		if (cqe->result != 0) {
			printk("async read failed %d\n", cqe->result);
			return;
		}

		rc = rtio_cqe_get_mempool_buffer(&stream_ctx, cqe, &buf, &buf_len);

		if (rc != 0) {
			printk("get mempool buffer failed %d\n", rc);
			return;
		}

		const struct device *sensor = dev;

		rtio_cqe_release(&stream_ctx, cqe);

		rc = sensor_get_decoder(sensor, &decoder);

		if (rc != 0) {
			printk("sensor_get_decoder failed %d\n", rc);
			return;
		}

		/* Frame iterator values when data comes from a FIFO */
		uint32_t accel_fit = 0;
		uint32_t pedo_fit = 0;

		/* Number of sensor data frames */
		uint16_t frame_count = 0, xl_count = 0;
		uint16_t pedo_count = 0;

		rc = decoder->get_frame_count(buf, accel_chan, &xl_count);
		rc += decoder->get_frame_count(buf, pedo_chan, &pedo_count);
		if (rc != 0) {
			printk("sensor_get_frame failed %d\n", rc);
			return;
		}

		frame_count = xl_count;
		frame_count += pedo_count;

		/* If a tap has occurred lets print it out */
		if (decoder->has_trigger(buf, SENSOR_TRIG_TAP)) {
			printk("Tap! Sensor %s\n", dev->name);
		}

		/* Decode all available sensor FIFO frames */
		printk("FIFO count: %d\n", frame_count);

		int i = 0;

		while (i < frame_count) {
			int8_t c = 0;

			/* decode and print Accelerometer FIFO frames */
			c = decoder->decode(buf, accel_chan, &accel_fit, 8, accel_data);

			for (int k = 0; k < c; k++) {
				printk("XL data for %s %lluns (%" PRIq(6) ", %" PRIq(6)
				       ", %" PRIq(6) ")\n", dev->name,
				       PRIsensor_three_axis_data_arg(*accel_data, k));
			}
			i += c;

			/* decode and print pedometer */
			c = decoder->decode(buf, pedo_chan, &pedo_fit, 8, pedo_data);

			for (int k = 0; k < c; k++) {
				printk("PEDO data for %s %lluns %s%d.%d\n", dev->name,
				       PRIsensor_q31_data_arg(*pedo_data, k));
			}
			i += c;
		}

		rtio_release_buffer(&stream_ctx, buf, buf_len);
	}
}

static void check_sensor_is_off(const struct device *dev)
{
	int ret;
	struct sensor_value odr;

	ret = sensor_attr_get(dev, SENSOR_CHAN_ACCEL_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

	/* Check if accel is off */
	if (ret != 0 || (odr.val1 == 0 && odr.val2 == 0)) {
		printk("%s WRN : accelerometer device is off\n", dev->name);
	}

	ret = sensor_attr_get(dev, SENSOR_CHAN_GYRO_XYZ, SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);

	/* Check if gyro is off */
	if (ret != 0 || (odr.val1 == 0 && odr.val2 == 0)) {
		printk("%s WRN : gyroscope device is off\n", dev->name);
	}
}

int main(void)
{
	struct sensor_value gbias[3];

	for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
		if (!device_is_ready(sensors[i])) {
			printk("sensor: device %s not ready.\n", sensors[i]->name);
			return 0;
		}
		check_sensor_is_off(sensors[i]);

		/*
		 * Set GBIAS as 0.5 rad/s, -1 rad/s, 0.2 rad/s
		 *
		 * (here application should initialize gbias x/y/z with latest values
		 * calculated from previous run and probably saved to non volatile memory)
		 */
		gbias[0].val1 = 0;
		gbias[0].val2 = 500000;
		gbias[1].val1 = -1;
		gbias[1].val2 = 0;
		gbias[2].val1 = 0;
		gbias[2].val2 = 200000;
		sensor_attr_set(sensors[i], SENSOR_CHAN_GBIAS_XYZ, SENSOR_ATTR_OFFSET, gbias);

		k_thread_create(&thread_id[i], thread_stack[i], TASK_STACK_SIZE, print_stream,
			(void *)sensors[i], (void *)iodevs[i], NULL, K_PRIO_COOP(5),
			K_INHERIT_PERMS, K_FOREVER);
		k_thread_start(&thread_id[i]);
	}

	return 0;
}
