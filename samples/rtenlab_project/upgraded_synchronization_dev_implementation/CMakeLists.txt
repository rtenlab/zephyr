# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(synchronization)

target_sources(app PRIVATE src/main.c
                            src/uart_i2c.c
                            src/battery/battery.c
                            src/sht31.c
                            src/apds9960.c
                            )
