# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(peripheral_esp)

target_sources(app PRIVATE

  src/main.c
  # src/uart_i2c.c

  src/battery/battery.c
  src/ds/Dallas_temperature.c
  src/ds/Onewire.c
)

zephyr_library_include_directories(${ZEPHYR_BASE}/samples/bluetooth)
