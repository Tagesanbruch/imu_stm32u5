# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(sc7a20h)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc)
FILE(GLOB app_sources src/*.c)
target_sources(app PRIVATE ${app_sources})
