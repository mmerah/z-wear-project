# Copyright (c) 2021 
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.13.1)
set(BOARD_ROOT ${CMAKE_CURRENT_LIST_DIR})

set(mcuboot_CONF_FILE
 ${CMAKE_CURRENT_SOURCE_DIR}/child_image/mcuboot.conf
)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(app LANGUAGES C VERSION 1.0.0)

target_include_directories(app PRIVATE ${CMAKE_BINARY_DIR}/app/include src)

FILE(GLOB main_sources src/*.c)
target_sources(app PRIVATE ${main_sources})

FILE(GLOB ble_sources src/ble/*.c)
target_sources(app PRIVATE ${ble_sources})

FILE(GLOB ble_services src/ble/services/*.c)
target_sources(app PRIVATE ${ble_services})

FILE(GLOB sensors_sources src/sensors/*.c)
target_sources(app PRIVATE ${sensors_sources})

FILE(GLOB battery_sources src/sensors/battery/*.c)
target_sources(app PRIVATE ${battery_sources})

add_subdirectory(drivers)

# Get current time
string(TIMESTAMP CURRENT_TIME %Y-%m-%dT%H:%M:%S)

# Version
set(SourceVersionFile "${CMAKE_CURRENT_SOURCE_DIR}/info/version.txt")
file(READ ${SourceVersionFile} ver)

string(REGEX MATCH "VERSION_MAJOR = ([0-9]*)" _ ${ver})
set(ZWEAR_VERSION_MAJOR ${CMAKE_MATCH_1})

string(REGEX MATCH "VERSION_MINOR = ([0-9]*)" _ ${ver})
set(ZWEAR_VERSION_MINOR ${CMAKE_MATCH_1})

string(REGEX MATCH "PATCHLEVEL = ([0-9]*)" _ ${ver})
set(ZWEAR_VERSION_PATCH ${CMAKE_MATCH_1})

string(REGEX MATCH "VERSION_TWEAK = ([0-9]*)" _ ${ver})
set(ZWEAR_VERSION_TWEAK ${CMAKE_MATCH_1})

# Get short commit hash
execute_process(
  COMMAND ${GIT_EXECUTABLE} describe --tags --dirty
  WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
  OUTPUT_VARIABLE FW_BUILD_GIT
  OUTPUT_STRIP_TRAILING_WHITESPACE
)
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -DCURRENT_TIME_OF_BUILD=\"${CURRENT_TIME}\" -DFW_BUILD=\"${FW_BUILD_GIT}\" -DSW_REVISION=\"${ZWEAR_VERSION_MAJOR}.${ZWEAR_VERSION_MINOR}.${ZWEAR_VERSION_PATCH}.${ZWEAR_VERSION_TWEAK}\"")
