add_executable(slac_unit_test
    libslac_unit_test.cpp
    qca7000_test.cpp
    qca7000_uart_test.cpp
    ../port/esp32s3/qca7000_link.cpp
    ../port/esp32s3/qca7000.cpp
    ../port/esp32s3/qca7000_uart.cpp
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

target_compile_definitions(slac_unit_test PRIVATE ARDUINO LIBSLAC_TESTING)

add_test(slac_unit_test slac_unit_test)
