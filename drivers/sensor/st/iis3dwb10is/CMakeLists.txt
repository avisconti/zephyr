# ST Microelectronics IIS3DWB10IS accelerometer sensor
#
# Copyright (c) 2025 STMicroelectronics
#
# SPDX-License-Identifier: Apache-2.0
#
zephyr_library()

zephyr_library_sources(iis3dwb10is.c)
zephyr_library_sources_ifdef(CONFIG_IIS3DWB10IS_TRIGGER  iis3dwb10is_trigger.c)
zephyr_library_sources_ifdef(CONFIG_SENSOR_ASYNC_API iis3dwb10is_decoder.c)
zephyr_library_sources_ifdef(CONFIG_IIS3DWB10IS_STREAM iis3dwb10is_stream.c)

zephyr_library_include_directories(../stmemsc)
