find_package(Threads REQUIRED)

add_subdirectory(evse)

add_executable(bridge bridge.cpp)
target_link_libraries(bridge PRIVATE
    Threads::Threads
    slac::slac
)
