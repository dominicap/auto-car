cmake_minimum_required(VERSION 3.13.3)
project(auto_car LANGUAGES C CXX ASM)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/Arduino-Toolchain.cmake)

get_board_id(board_id mega atmega2560)

add_arduino_executable(auto_car ${board_id} src/auto_car.cpp)

link_platform_library(auto_car Wire ${board_id})

add_arduino_library(adafruit_sensor ${board_id} libs/Adafruit_Sensor/src/Adafruit_Sensor.h)
add_arduino_library(adafruit_bno055 ${board_id} libs/Adafruit_BNO055/src/Adafruit_BNO055.h)
add_arduino_library(adafruit_vl53l0x ${board_id} libs/Adafruit_VL53L0X/src/Adafruit_VL53L0X.h)

link_arduino_library(auto_car adafruit_sensor ${board_id})
link_arduino_library(auto_car adafruit_bno055 ${board_id})
link_arduino_library(auto_car adafruit_vl53l0x ${board_id})
