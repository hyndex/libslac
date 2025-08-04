# When this repository is included as an ESP-IDF component the build
# system first parses this file in "script mode" which only allows a
# limited set of commands.  Avoid running the normal project setup in
# that case to prevent errors such as "define_property command is not
# scriptable".
if(CMAKE_SCRIPT_MODE_FILE)
    return()
endif()

cmake_minimum_required(VERSION 3.16.0)
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(libslac)
