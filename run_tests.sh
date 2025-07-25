#!/bin/sh
set -e

CXXFLAGS="-std=c++17 -DNDEBUG -DLIBSLAC_TESTING -DARDUINO -Iinclude -I3rd_party -I3rd_party/fsm -Iport/esp32s3 -Iport -Itests -I. -Wduplicated-cond -Wduplicated-branches"
SRCS="tests/test_endian.cpp tests/test_sha256.cpp tests/test_payload.cpp tests/test_channel.cpp tests/test_fsm.cpp tests/test_fsm_buffer.cpp tests/test_qca7000_link.cpp tests/test_reset.cpp tests/test_check_alive.cpp tests/qca7000_hal_mock.cpp src/channel.cpp src/slac.cpp port/esp32s3/qca7000_link.cpp 3rd_party/hash_library/sha256.cpp"


#g++ $CXXFLAGS $SRCS -lgtest -lgtest_main -pthread -o tests_run

g++ $CXXFLAGS $SRCS -lgtest -lgtest_main -pthread -o tests_run
./tests_run
