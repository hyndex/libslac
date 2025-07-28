#!/bin/sh
set -e

# Abort early when GoogleTest headers are missing.  The unit tests rely on
# the system provided gtest development package (e.g. ``libgtest-dev`` on
# Debian/Ubuntu).
if [ ! -f /usr/include/gtest/gtest.h ] && [ ! -f /usr/local/include/gtest/gtest.h ]; then
    echo "GoogleTest headers not found. Attempting to install libgtest-dev..." >&2
    if command -v apt-get >/dev/null 2>&1; then
        sudo apt-get update && sudo apt-get install -y libgtest-dev
    else
        echo "apt-get not available. Please install libgtest-dev manually." >&2
        exit 1
    fi
fi

CXXFLAGS="-std=c++17 -DNDEBUG -DLIBSLAC_TESTING -DARDUINO -DCP_USE_DMA_ADC=1 -DportTICK_PERIOD_MS=1 -Iinclude -I3rd_party -I3rd_party/fsm -Iport/esp32s3 -Iport -Itests -Iexamples/platformio_complete/src -I. -Wduplicated-cond -Wduplicated-branches"
SRCS="tests/test_endian.cpp tests/test_sha256.cpp tests/test_nid.cpp tests/test_payload.cpp tests/test_channel.cpp tests/test_fsm.cpp tests/test_fsm_buffer.cpp tests/test_qca7000_link.cpp tests/test_reset.cpp tests/test_check_alive.cpp tests/test_rx_ring.cpp tests/test_slac_retry.cpp tests/test_bcb_wakeup.cpp tests/test_slac_filter.cpp tests/test_validation.cpp tests/test_qca7000_fetch_rx.cpp tests/test_cp_monitor.cpp tests/cp_monitor_mocks.cpp tests/time_mock.cpp tests/qca7000_hal_mock.cpp src/channel.cpp src/slac.cpp src/config.cpp src/match_log.cpp examples/platformio_complete/src/cp_monitor.cpp port/esp32s3/qca7000_link.cpp 3rd_party/hash_library/sha256.cpp"


#g++ $CXXFLAGS $SRCS -lgtest -lgtest_main -pthread -o tests_run

g++ $CXXFLAGS $SRCS -lgtest -lgtest_main -pthread -o tests_run
./tests_run
