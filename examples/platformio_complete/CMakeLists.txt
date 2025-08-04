cmake_minimum_required(VERSION 3.16.0)

# When ESP-IDF scans this file in "script-mode" it expects no regular
# project commands. Guard the real configuration so that only the second
# pass executes it. This avoids errors like "define_property command is
# not scriptable" that stem from ESP-IDF loading CMake modules such as
# ExternalProject during the early pass.
if(CMAKE_SCRIPT_MODE_FILE)
    return()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(platformio_complete)
