# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(peripheral_esp)

target_sources(app PRIVATE
  src/main.c
  src/uart_i2c.c
  src/sht31.c
  src/apds9960.c
  src/bmp280.c
  src/lsm6ds33.c
)

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
