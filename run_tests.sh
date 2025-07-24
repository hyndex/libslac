#!/bin/sh
set -e
CXXFLAGS="-std=c++17 -DNDEBUG -Iinclude -I3rd_party -Iport/esp32s3 -Iport -I."
SRCS="tests/test_endian.cpp tests/test_sha256.cpp tests/test_payload.cpp tests/test_channel.cpp src/channel.cpp src/slac.cpp 3rd_party/hash_library/sha256.cpp"

#g++ $CXXFLAGS $SRCS -lgtest -lgtest_main -pthread -o tests_run

g++ $CXXFLAGS $SRCS -lgtest -lgtest_main -pthread -o tests_run
./tests_run
