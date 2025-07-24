
add_executable(evse
    evse.cpp
    evse_fsm.cpp
    slac_io.cpp
    packet_socket.cpp
    packet_socket_link.cpp
)

target_link_libraries(evse PRIVATE
    Threads::Threads
    libfsm
    slac::slac
)
