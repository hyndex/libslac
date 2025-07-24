add_executable(slac_unit_test
    libslac_unit_test.cpp
    evse_fsm_test.cpp
    qca7000_test.cpp
    qca7000_uart_test.cpp
    ../tools/evse/evse_fsm.cpp
    ../tools/evse/slac_io.cpp
    stubs/SPI.cpp
    stubs/HardwareSerial.cpp
)

find_package(GTest REQUIRED)

set(GTEST_LIBRARIES
    GTest::gtest
    GTest::gtest_main
)

target_include_directories(slac_unit_test PUBLIC
    ${GTEST_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/../3rd_party/libfsm
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools/evse
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
    ${CMAKE_CURRENT_SOURCE_DIR}/../port/esp32s3
    ${CMAKE_CURRENT_SOURCE_DIR}/../port/generic
    ${CMAKE_CURRENT_SOURCE_DIR}/..
)

target_link_libraries(slac_unit_test PRIVATE
    ${GTEST_LIBRARIES}
    slac::slac
    libfsm
    Threads::Threads
)

add_test(slac_unit_test slac_unit_test)
