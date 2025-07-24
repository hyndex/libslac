
add_executable(evse
    evse.cpp
    evse_fsm.cpp
    slac_io.cpp
)

target_link_libraries(evse PRIVATE
    Threads::Threads
    libfsm
    slac::slac
)
