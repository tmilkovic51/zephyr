# SPDX-License-Identifier: Apache-2.0

zephyr_library()

# Include drivers directory - needed for STM32 clock control driver calls in power_states.c
zephyr_library_include_directories(${ZEPHYR_BASE}/drivers)

# Don't emit a cmake warning if CONFIG_PM is not enabled
zephyr_library_property(ALLOW_EMPTY TRUE)

zephyr_library_sources_ifdef(CONFIG_PM power_states/power_states.c)
