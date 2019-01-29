cmake_minimum_required(VERSION 3.13.2)

set(CMAKE_TOOLCHAIN_FILE ${CMAKE_SOURCE_DIR}/cmake/ArduinoToolchain.cmake)
set(CMAKE_CXX_STANDARD 14)
set(PROJECT_NAME AutoCar)

set(${PROJECT_NAME}_SRCS main.cpp)
set(${PROJECT_NAME}_BOARD uno)
set(${PROJECT_NAME}_PORT /dev/ttyACM0)
set(${PROJECT_NAME}_SERIAL screen @SERIAL_PORT@ 9600)

link_directories(${CMAKE_CURRENT_SOURCE_DIR}/libraries)

project(${PROJECT_NAME} C CXX)

generate_arduino_firmware(${PROJECT_NAME})
