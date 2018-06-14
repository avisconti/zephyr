zephyr_library()

zephyr_library_sources_ifdef(CONFIG_I2S_SAM_SSC		i2s_sam_ssc.c)
zephyr_library_sources_ifdef(CONFIG_I2S_CAVS		i2s_cavs.c)
zephyr_library_sources_ifdef(CONFIG_I2S_STM32		i2s_ll_stm32.c)
