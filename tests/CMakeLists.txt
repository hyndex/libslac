add_executable(${PROJECT_NAME}_unit_test libslac_unit_test.cpp)

find_package(GTest REQUIRED)
target_include_directories(${PROJECT_NAME}_unit_test PUBLIC ${GTEST_INCLUDE_DIRS})

set(GTEST_LIBRARIES
    GTest::gtest
    GTest::gtest_main
)

target_link_libraries(${PROJECT_NAME}_unit_test PRIVATE
        ${GTEST_LIBRARIES}
        slac::slac
)

add_test(${PROJECT_NAME}_unit_test ${PROJECT_NAME}_unit_test)
