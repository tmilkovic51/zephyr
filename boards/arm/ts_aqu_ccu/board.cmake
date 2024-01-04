# Copyright (c) 2023 Byte Lab d.o.o. <dev@byte-lab.com>
# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=STM32H7A3VI" "--speed=4000")

include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
