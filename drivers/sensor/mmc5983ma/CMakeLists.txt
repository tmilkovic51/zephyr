# SPDX-License-Identifier: Apache-2.0

zephyr_library()

zephyr_library_sources(mmc5983ma.c)
zephyr_library_sources_ifdef(CONFIG_MMC5983MA_TRIGGER mmc5983ma_trigger.c)
