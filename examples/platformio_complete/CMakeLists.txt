# Guard against the ESP-IDF "script mode" pass which executes this
# file with a reduced set of commands.  Only proceed with the full
# project setup during the normal configure step.
if(CMAKE_SCRIPT_MODE_FILE)
    return()
endif()

cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(platformio_complete)
