/*
 * Copyright (c) 2018 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <misc/printk.h>

#include <board.h>
#include <gpio.h>
#include <led.h>
#include <i2s.h>
#include <i2c.h>
#include <spi.h>
#include <sensor.h>

#ifdef CONFIG_I2S
#include "OpenPDMFilter.h"
#endif

/* #define ARGONKEY_TEST_LOG 1 */

#define WHOAMI_REG      0x0F
#define WHOAMI_ALT_REG  0x4F

static inline float out_ev(struct sensor_value *val)
{
	return (val->val1 + (float)val->val2 / 1000000);
}

static int lsm6dsl_trig_cnt;
#ifdef CONFIG_LSM6DSL_TRIGGER
static void lsm6dsl_trigger_handler(struct device *dev,
				    struct sensor_trigger *trig)
{
#ifdef ARGONKEY_TEST_LOG
	char out_str[64];
#endif
	struct sensor_value accel_x, accel_y, accel_z;
	struct sensor_value gyro_x, gyro_y, gyro_z;

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
	struct sensor_value magn_x, magn_y, magn_z;
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
	struct sensor_value press, temp;
#endif

	lsm6dsl_trig_cnt++;

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_ACCEL_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_X, &accel_x);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
	sensor_channel_get(dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
#ifdef ARGONKEY_TEST_LOG
	sprintf(out_str, "accel (%f %f %f) m/s2", out_ev(&accel_x),
						out_ev(&accel_y),
						out_ev(&accel_z));
	printk("TRIG %s\n", out_str);
#endif

	/* lsm6dsl gyro */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_GYRO_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_X, &gyro_x);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
	sensor_channel_get(dev, SENSOR_CHAN_GYRO_Z, &gyro_z);
#ifdef ARGONKEY_TEST_LOG
	sprintf(out_str, "gyro (%f %f %f) dps", out_ev(&gyro_x),
						out_ev(&gyro_y),
						out_ev(&gyro_z));
	printk("TRIG %s\n", out_str);
#endif

#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
	/* lsm6dsl magn */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_MAGN_XYZ);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_X, &magn_x);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Y, &magn_y);
	sensor_channel_get(dev, SENSOR_CHAN_MAGN_Z, &magn_z);
#ifdef ARGONKEY_TEST_LOG
	sprintf(out_str, "magn (%f %f %f) gauss", out_ev(&magn_x),
						 out_ev(&magn_y),
						 out_ev(&magn_z));
	printk("TRIG %s\n", out_str);
#endif

#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
	/* lsm6dsl press/temp */
	sensor_sample_fetch_chan(dev, SENSOR_CHAN_PRESS);
	sensor_channel_get(dev, SENSOR_CHAN_PRESS, &press);

	sensor_sample_fetch_chan(dev, SENSOR_CHAN_AMBIENT_TEMP);
	sensor_channel_get(dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

#ifdef ARGONKEY_TEST_LOG
	sprintf(out_str, "press (%f) kPa - temp (%f) deg", out_ev(&press),
							   out_ev(&temp));
	printk("%s\n", out_str);
#endif

#endif
}
#endif

#ifdef CONFIG_I2S
#define AUDIO_FREQ		32000
#define OVERSAMPLING_FACTOR	64
#define CHAN_SIZE		16
#define NUM_RX_BLOCKS		4
#define NUM_TX_BLOCKS		4
#define BLOCK_SIZE		(AUDIO_FREQ/1000)*OVERSAMPLING_FACTOR/CHAN_SIZE
#define BLOCK_SIZE_BYTES	(BLOCK_SIZE * sizeof(s16_t))

K_MEM_SLAB_DEFINE(rx_mem_slab, BLOCK_SIZE_BYTES, NUM_RX_BLOCKS, 1);
K_MEM_SLAB_DEFINE(tx_mem_slab, BLOCK_SIZE_BYTES, NUM_TX_BLOCKS, 1);

#define NUM_MS		3000
u16_t pcm_out[32*NUM_MS];
#endif

#define NUM_LEDS 12
#define DELAY_TIME K_MSEC(50)

void main(void)
{
	int cnt = 0;
	char out_str[64];
	static struct device *led0, *led1;
	int i, on = 1;

#ifdef CONFIG_LP3943
	static struct device *ledc;

	ledc = device_get_binding(CONFIG_LP3943_DEV_NAME);
	if (!ledc) {
		printk("Could not get pointer to %s sensor\n",
			CONFIG_LP3943_DEV_NAME);
		return;
	}

	/* turn all leds on */
	for (i = 0; i < NUM_LEDS; i++) {
		led_on(ledc, i);
		k_sleep(DELAY_TIME);
	}

	/* turn all leds off */
	for (i = 0; i < NUM_LEDS; i++) {
		led_off(ledc, i);
		k_sleep(DELAY_TIME);
	}
#endif

	led0 = device_get_binding(LED0_GPIO_CONTROLLER);
	gpio_pin_configure(led0, LED0_GPIO_PIN, GPIO_DIR_OUT);
	gpio_pin_write(led0, LED0_GPIO_PIN, 1);

	led1 = device_get_binding(LED1_GPIO_CONTROLLER);
	gpio_pin_configure(led1, LED1_GPIO_PIN, GPIO_DIR_OUT);

	for (i = 0; i < 5; i++) {
		gpio_pin_write(led1, LED1_GPIO_PIN, on);
		k_sleep(200);
		on = (on == 1) ? 0 : 1;
	}

	printk("ArgonKey test!!\n");

#ifdef CONFIG_I2S
	struct i2s_config i2s_cfg;
	int a_ret;
	struct device *adev;
	void *rx_block;
	size_t rx_size;

#ifdef CONFIG_I2S_1
	struct device *adev1 = device_get_binding(CONFIG_I2S_1_NAME);

	if (!adev1) {
		printk("Could not get pointer to %s sensor\n",
			CONFIG_I2S_1_NAME);
		return;
	}
#endif
#ifdef CONFIG_I2S_4
	struct device *adev4 = device_get_binding(CONFIG_I2S_4_NAME);

	if (!adev4) {
		printk("Could not get pointer to %s sensor\n",
			CONFIG_I2S_4_NAME);
		return;
	}
#endif
#ifdef CONFIG_I2S_5
	struct device *adev5 = device_get_binding(CONFIG_I2S_5_NAME);

	if (!adev5) {
		printk("Could not get pointer to %s sensor\n",
			CONFIG_I2S_5_NAME);
		return;
	}
#endif

	adev = adev5;

	i2s_cfg.word_size = CHAN_SIZE;
	i2s_cfg.channels = 1;
	i2s_cfg.format = I2S_FMT_DATA_FORMAT_LEFT_JUSTIFIED;
	i2s_cfg.options = I2S_OPT_FRAME_CLK_MASTER | I2S_OPT_BIT_CLK_MASTER;
	i2s_cfg.frame_clk_freq = AUDIO_FREQ * OVERSAMPLING_FACTOR / CHAN_SIZE;
	i2s_cfg.block_size = BLOCK_SIZE * sizeof(s16_t);
	i2s_cfg.mem_slab = &rx_mem_slab;
	i2s_cfg.timeout = 2000;

	#if 0
	int j;
	void *tx_block;
	i2s_cfg.mem_slab = &tx_mem_slab;

	/* TX part */
	a_ret = i2s_configure(adev, I2S_DIR_TX, &i2s_cfg);
	if (a_ret != 0) {
		printk("i2s configuration failed with %d error\n", a_ret);
		return;
	}

	for (j = 0; j < 3; j++) {
		a_ret = k_mem_slab_alloc(&tx_mem_slab, &tx_block, K_FOREVER);
		if (a_ret < 0) {
			printk("Error: failed to allocate tx_block\n");
			return ;
		}

		for (i = 0; i < BLOCK_SIZE_BYTES/2; i++)
			((u16_t *)tx_block)[i] = (i + j) & 0xffff;

		a_ret = i2s_write(adev, tx_block, BLOCK_SIZE_BYTES);
		if (a_ret < 0) {
			k_mem_slab_free(&tx_mem_slab, &tx_block);
		}
	}

	/* Start transmission */
	a_ret = i2s_trigger(adev, I2S_DIR_TX, I2S_TRIGGER_START);
	if (a_ret != 0) {
		printk("Start Trigger failed with %d error\n", a_ret);
		return;
	}
	while(1);

	while(1) {
		a_ret = k_mem_slab_alloc(&tx_mem_slab, &tx_block, K_FOREVER);
		if (a_ret < 0) {
			printk("Error: failed to allocate tx_block\n");
			return ;
		}

		for (i = 0; i < BLOCK_SIZE_BYTES; i++)
			((u8_t *)tx_block)[i] = i & 0xff;

		a_ret = i2s_write(adev, tx_block, BLOCK_SIZE_BYTES);
		if (a_ret < 0) {
			k_mem_slab_free(&tx_mem_slab, &tx_block);
		}
	}
	#endif

	/* init the filter lib */
	TPDMFilter_InitStruct filter_init;

	filter_init.LP_HZ = AUDIO_FREQ / 2;
	filter_init.HP_HZ = 10;
	filter_init.Fs = AUDIO_FREQ;
	filter_init.Out_MicChannels = 1;
	filter_init.In_MicChannels = 1;
	filter_init.Decimation = OVERSAMPLING_FACTOR;

	Open_PDM_Filter_Init(&filter_init);

	u32_t ms_count;

	for (i = 0; i < 32*NUM_MS; i++)
		pcm_out[i] = 0x0;

	/* RX part */
	a_ret = i2s_configure(adev, I2S_DIR_RX, &i2s_cfg);
	if (a_ret != 0) {
		printk("i2s configuration failed with %d error\n", a_ret);
		return;
	}

	a_ret = i2s_trigger(adev, I2S_DIR_RX, I2S_TRIGGER_START);
	if (a_ret != 0) {
		printk("i2s trigger failed with %d error\n", a_ret);
		return;
	}

	ms_count = 0;
	while (1) {
		a_ret = i2s_read(adev, &rx_block, &rx_size);
		if (a_ret != 0) {
			printk("i2s read failed with %d error\n", a_ret);
			//break;
			return;
		}

		Open_PDM_Filter_64((u8_t *) rx_block, pcm_out + 32*ms_count, 64, &filter_init);
		k_mem_slab_free(&rx_mem_slab, &rx_block);

		if (ms_count++ >= NUM_MS)
			break;
	}

	/* print PCM stream */
	printk("--\n");
	for (i = 0; i < 32*NUM_MS; i++)
		printk("0x%04x,\n", pcm_out[i]);

	printk("okkk!\n");
	while(1);

	/* print PDM stream */
	{
	    int i;

	    for (i = 0; i < rx_size; i += 2)
		printk("0x%04x,\n", ((u16_t *)rx_block)[i]);
	}
	printk("okkk!\n");
	while(1);
#endif

#ifdef CONFIG_LPS22HB
	struct device *baro_dev = device_get_binding(CONFIG_LPS22HB_DEV_NAME);

	if (!baro_dev) {
		printk("Could not get pointer to %s sensor\n",
			CONFIG_LPS22HB_DEV_NAME);
		return;
	}
#endif

#ifdef CONFIG_HTS221
	struct device *hum_dev = device_get_binding(CONFIG_HTS221_NAME);

	if (!hum_dev) {
		printk("Could not get pointer to %s sensor\n",
			CONFIG_HTS221_NAME);
		return;
	}
#endif

#ifdef CONFIG_LSM6DSL
	struct device *accel_dev = device_get_binding(CONFIG_LSM6DSL_DEV_NAME);

	if (!accel_dev) {
		printk("Could not get pointer to %s sensor\n",
			CONFIG_LSM6DSL_DEV_NAME);
		return;
	}

#if defined(CONFIG_LSM6DSL_ACCEL_ODR) && (CONFIG_LSM6DSL_ACCEL_ODR == 0)
	struct sensor_value a_odr_attr;

	/* set sampling frequency to 104Hz for accel */
	a_odr_attr.val1 = 104;
	a_odr_attr.val2 = 0;

	if (sensor_attr_set(accel_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &a_odr_attr) < 0) {
		printk("Cannot set sampling frequency for accelerometer.\n");
		return;
	}
#endif

#if defined(CONFIG_LSM6DSL_ACCEL_FS) && (CONFIG_LSM6DSL_ACCEL_FS == 0)
	struct sensor_value a_fs_attr;

	/* set full scale to 16g for accel */
	sensor_g_to_ms2(16, &a_fs_attr);

	if (sensor_attr_set(accel_dev, SENSOR_CHAN_ACCEL_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &a_fs_attr) < 0) {
		printk("Cannot set fs for accelerometer.\n");
		return;
	}
#endif

#if defined(CONFIG_LSM6DSL_GYRO_ODR) && (CONFIG_LSM6DSL_GYRO_ODR == 0)
	struct sensor_value g_odr_attr;

	/* set sampling frequency to 104Hz for accel */
	g_odr_attr.val1 = 104;
	g_odr_attr.val2 = 0;

	if (sensor_attr_set(accel_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_SAMPLING_FREQUENCY, &g_odr_attr) < 0) {
		printk("Cannot set sampling frequency for gyro.\n");
		return;
	}
#endif

#if defined(CONFIG_LSM6DSL_GYRO_FS) && (CONFIG_LSM6DSL_GYRO_FS == 0)
	struct sensor_value g_fs_attr;

	/* set full scale to 245dps for accel */
	sensor_g_to_ms2(245, &g_fs_attr);

	if (sensor_attr_set(accel_dev, SENSOR_CHAN_GYRO_XYZ,
			    SENSOR_ATTR_FULL_SCALE, &g_fs_attr) < 0) {
		printk("Cannot set fs for gyroscope.\n");
		return;
	}
#endif

#endif

#ifdef CONFIG_VL53L0X
	struct device *tof_dev = device_get_binding(CONFIG_VL53L0X_NAME);

	if (!tof_dev) {
		printk("Could not get pointer to %s sensor\n",
			CONFIG_VL53L0X_NAME);
		return;
	}
#endif

#ifdef CONFIG_LSM6DSL_TRIGGER
	struct sensor_trigger trig;

	trig.type = SENSOR_TRIG_DATA_READY;
	trig.chan = SENSOR_CHAN_ACCEL_XYZ;
	sensor_trigger_set(accel_dev, &trig, lsm6dsl_trigger_handler);
#endif

	while (1) {
#ifdef CONFIG_LPS22HB
		struct sensor_value temp, press;
#endif
#ifdef CONFIG_HTS221
		struct sensor_value humidity;
#endif
#ifdef CONFIG_LSM6DSL
		struct sensor_value accel_x, accel_y, accel_z;
		struct sensor_value gyro_x, gyro_y, gyro_z;
#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
		struct sensor_value magn_x, magn_y, magn_z;
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
		struct sensor_value press, temp;
#endif
#endif
#ifdef CONFIG_VL53L0X
		struct sensor_value prox;
#endif

#ifdef CONFIG_VL53L0X
		sensor_sample_fetch(tof_dev);
		sensor_channel_get(tof_dev, SENSOR_CHAN_PROX, &prox);
		printk("proxy: %d  ;\n", prox.val1);
		sensor_channel_get(tof_dev, SENSOR_CHAN_DISTANCE, &prox);
		printk("distance: %d -- %3d mm;\n", prox.val1, prox.val2);
#endif

#ifdef CONFIG_LPS22HB
		sensor_sample_fetch(baro_dev);
		sensor_channel_get(baro_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		sensor_channel_get(baro_dev, SENSOR_CHAN_PRESS, &press);

		printk("temp: %d.%02d C; press: %d.%06d\n",
		       temp.val1, temp.val2, press.val1, press.val2);
#endif

#ifdef CONFIG_HTS221
		sensor_sample_fetch(hum_dev);
		sensor_channel_get(hum_dev, SENSOR_CHAN_HUMIDITY, &humidity);

		printk("humidity: %d.%06d\n",
		       humidity.val1, humidity.val2);
#endif

#ifdef CONFIG_LSM6DSL
		/* lsm6dsl accel */
		sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_ACCEL_XYZ);
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_X, &accel_x);
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Y, &accel_y);
		sensor_channel_get(accel_dev, SENSOR_CHAN_ACCEL_Z, &accel_z);
		sprintf(out_str, "accel (%f %f %f) m/s2", out_ev(&accel_x),
							out_ev(&accel_y),
							out_ev(&accel_z));
		printk("%s\n", out_str);

		/* lsm6dsl gyro */
		sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_GYRO_XYZ);
		sensor_channel_get(accel_dev, SENSOR_CHAN_GYRO_X, &gyro_x);
		sensor_channel_get(accel_dev, SENSOR_CHAN_GYRO_Y, &gyro_y);
		sensor_channel_get(accel_dev, SENSOR_CHAN_GYRO_Z, &gyro_z);
		sprintf(out_str, "gyro (%f %f %f) dps", out_ev(&gyro_x),
							out_ev(&gyro_y),
							out_ev(&gyro_z));
		printk("%s\n", out_str);
#if defined(CONFIG_LSM6DSL_EXT0_LIS2MDL)
		/* lsm6dsl magn */
		sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_MAGN_XYZ);
		sensor_channel_get(accel_dev, SENSOR_CHAN_MAGN_X, &magn_x);
		sensor_channel_get(accel_dev, SENSOR_CHAN_MAGN_Y, &magn_y);
		sensor_channel_get(accel_dev, SENSOR_CHAN_MAGN_Z, &magn_z);
		sprintf(out_str, "magn (%f %f %f) gauss", out_ev(&magn_x),
							 out_ev(&magn_y),
							 out_ev(&magn_z));
		printk("%s\n", out_str);
#endif
#if defined(CONFIG_LSM6DSL_EXT0_LPS22HB)
		/* lsm6dsl press/temp */
		sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_PRESS);
		sensor_channel_get(accel_dev, SENSOR_CHAN_PRESS, &press);

		sensor_sample_fetch_chan(accel_dev, SENSOR_CHAN_AMBIENT_TEMP);
		sensor_channel_get(accel_dev, SENSOR_CHAN_AMBIENT_TEMP, &temp);

		sprintf(out_str, "press (%f) kPa - temp (%f) deg",
			out_ev(&press), out_ev(&temp));
		printk("%s\n", out_str);
#endif

#endif /* CONFIG_LSM6DSL */

		printk("- (%d) (trig_cnt: %d)\n\n", ++cnt, lsm6dsl_trig_cnt);
		k_sleep(2000);
	}
}


