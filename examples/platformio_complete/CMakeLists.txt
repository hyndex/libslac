cmake_minimum_required(VERSION 3.16.0)
# When this file is parsed by ESP-IDF's component manager in script mode, it
# must not include project.cmake or call project(), otherwise CMake raises
# "define_property command is not scriptable" errors. Guard both calls so they
# only execute during normal builds.
if(NOT DEFINED CMAKE_SCRIPT_MODE_FILE)
    include($ENV{IDF_PATH}/tools/cmake/project.cmake)
    project(platformio_complete)
endif()
