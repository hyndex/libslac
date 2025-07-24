cmake_minimum_required(VERSION 3.11)

project(slac VERSION 0.3.1
        DESCRIPTION "Simple ISO15118-3 SLAC implementation"
		LANGUAGES CXX C
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# options
option(BUILD_SLAC_TOOLS "Build SLAC tools" OFF)
set(SLAC_INSTALL_DEFAULT OFF)
if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
  set(SLAC_INSTALL_DEFAULT ON)
endif()
option(SLAC_INSTALL "Install the library (shared data might be installed anyway)" ${SLAC_INSTALL_DEFAULT})
option(${PROJECT_NAME}_BUILD_TESTING "Build unit tests, used if included as dependency" OFF)
option(BUILD_TESTING "Build unit tests, used if standalone project" OFF)

if(ESP_PLATFORM)
  set(BUILD_SLAC_TOOLS OFF CACHE BOOL "Build SLAC tools" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "Build unit tests" FORCE)
endif()

if(BUILD_TESTING)
    if(CMAKE_PROJECT_NAME STREQUAL PROJECT_NAME OR ${PROJECT_NAME}_BUILD_TESTING)
        set(LIBSLAC_BUILD_TESTING ON)
        include(CTest)
        add_subdirectory(tests)
    endif()
endif()


add_subdirectory(3rd_party)

add_library(slac)
add_library(slac::slac ALIAS slac)

target_include_directories(slac PRIVATE
    $<TARGET_PROPERTY:HashLibrary,INTERFACE_INCLUDE_DIRECTORIES>
    ${CMAKE_CURRENT_SOURCE_DIR}/src
)

target_include_directories(slac PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/port/generic>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>
    $<INSTALL_INTERFACE:include>
)

set(SLAC_SOURCES
    src/channel.cpp
    src/slac.cpp
)
option(SLAC_USE_UART "Use UART interface for QCA7000" OFF)

if (ESP_PLATFORM)
    if (SLAC_USE_UART)
        list(APPEND SLAC_SOURCES
            port/esp32s3/qca7000_uart.cpp
        )
    else()
        list(APPEND SLAC_SOURCES
            port/esp32s3/qca7000_link.cpp
            port/esp32s3/qca7000.cpp
        )
    endif()
else()
    list(APPEND SLAC_SOURCES
        src/packet_socket.cpp
        src/packet_socket_link.cpp
    )
endif()

target_sources(slac PRIVATE ${SLAC_SOURCES} $<TARGET_OBJECTS:HashLibrary>)
if (SLAC_USE_UART)
    target_compile_definitions(slac PUBLIC SLAC_USE_UART)
endif()

if (ESP_PLATFORM)
    target_include_directories(slac PRIVATE port/esp32s3)
endif()
target_compile_options(slac PRIVATE -Os -fdata-sections -ffunction-sections -fno-exceptions -fno-rtti)

if (BUILD_SLAC_TOOLS)
    add_subdirectory(tools)
endif()


if (SLAC_INSTALL)
    install(TARGETS slac
            EXPORT slac-targets
            ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
            LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
            RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})

    install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

    install(EXPORT slac-targets
            NAMESPACE slac::
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/slac)

    write_basic_package_version_file(
        ${CMAKE_CURRENT_BINARY_DIR}/slac-config-version.cmake
        VERSION ${PROJECT_VERSION}
        COMPATIBILITY SameMajorVersion)

    configure_package_config_file(
        ${CMAKE_CURRENT_LIST_DIR}/cmake/slac-config.cmake.in
        ${CMAKE_CURRENT_BINARY_DIR}/slac-config.cmake
        INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/slac)

    install(FILES
            ${CMAKE_CURRENT_BINARY_DIR}/slac-config.cmake
            ${CMAKE_CURRENT_BINARY_DIR}/slac-config-version.cmake
            DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/slac)
endif()

export(TARGETS slac NAMESPACE slac:: FILE ${CMAKE_CURRENT_BINARY_DIR}/slac-targets.cmake)

