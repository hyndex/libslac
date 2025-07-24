add_library(HashLibrary OBJECT
    hash_library/sha256.cpp
)

target_include_directories(HashLibrary PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../port/generic>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/..>
)

# header-only libfsm
add_library(libfsm INTERFACE)
target_include_directories(libfsm INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/libfsm>
)
