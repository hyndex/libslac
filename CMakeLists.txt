cmake_minimum_required(VERSION 3.16.0)

# Prevent ESP-IDF's early "script-mode" pass from evaluating project
# commands or pulling in heavy modules. Only the real configure step
# should include the IDF build system.
if(CMAKE_SCRIPT_MODE_FILE)
    return()
endif()

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(libslac)
