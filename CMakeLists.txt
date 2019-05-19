cmake_minimum_required(VERSION 3.14)
project(AutoCar LANGUAGES C CXX ASM)

get_board_id(board_id mega atmega2560)

add_arduino_executable(auto_car ${board_id} src/auto_car.cpp)

find_arduino_library(servo Servo ${board_id})
link_arduino_library(auto_car servo ${board_id})

add_arduino_header_only_library(adafruit_sensor ${board_id} libraries/Adafruit_Sensor/src/Adafruit_Sensor.h)

find_arduino_library(adafruit_bno055 Adafruit_BNO055 ${board_id} 3RD_PARTY)
find_arduino_library(adafruit_vl53l0x Adafruit_VL53L0X ${board_id} 3RD_PARTY)

link_platform_library(adafruit_bno055 Wire ${board_id})
link_platform_library(adafruit_vl53l0x Wire ${board_id})

link_arduino_library(adafruit_bno055 adafruit_sensor ${board_id} HEADER_ONLY)

link_arduino_library(auto_car adafruit_bno055 ${board_id})
link_arduino_library(auto_car adafruit_vl53l0x ${board_id})