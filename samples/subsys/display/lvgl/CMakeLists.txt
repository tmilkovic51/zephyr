# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(lvgl)

FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources} ../../../../../modules/lib/gui/lvgl/demos/widgets/lv_demo_widgets.c ../../../../../modules/lib/gui/lvgl/demos/widgets/assets/img_clothes.c ../../../../../modules/lib/gui/lvgl/demos/widgets/assets/img_lvgl_logo.c ../../../../../modules/lib/gui/lvgl/demos/widgets/assets/img_demo_widgets_avatar.c)
