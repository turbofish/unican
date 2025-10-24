# Unican - Universal CAN interface Library
# CMake configuration for building the unican library
# Supports multiple CAN interfaces with optional compilation

cmake_minimum_required(VERSION 3.15)
project(unican VERSION 1.0.0 LANGUAGES CXX)

# Require C++20 (for std::span support)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Build options - user can enable/disable backends
# These options control which CAN backends are compiled into the library
option(ENABLE_SOCKETCAN "Enable SocketCAN backend (Linux native CAN)" ON)
option(ENABLE_KVASER "Enable Kvaser backend (Kvaser CANlib)" ON)
option(ENABLE_PCAN "Enable PCAN backend (PEAK-System PCAN)" OFF)
option(ENABLE_IXXAT "Enable IXXAT backend (IXXAT CAN interfaces)" OFF)
option(BUILD_EXAMPLES "Build example programs and demos" ON)

# Core source files - always compiled
# These files contain the main library functionality
set(UNICAN_SOURCES
    can_frame.cpp
    can_bus.cpp
    bus_null.cpp
)

# Add backend sources based on enabled options
# Each backend is conditionally compiled based on user options
if(ENABLE_SOCKETCAN)
    if(UNIX AND NOT APPLE)  # SocketCAN is Linux-specific
        list(APPEND UNICAN_SOURCES bus_socketcan.cpp)
        add_compile_definitions(UNICAN_HAS_SOCKETCAN)
    else()
        message(WARNING "SocketCAN is only available on Linux systems")
    endif()
endif()

if(ENABLE_KVASER)
    list(APPEND UNICAN_SOURCES bus_kvaser.cpp)
    add_compile_definitions(UNICAN_HAS_KVASER)
endif()

if(ENABLE_PCAN)
    list(APPEND UNICAN_SOURCES pcan.cpp)
    add_compile_definitions(UNICAN_HAS_PCAN)
endif()

if(ENABLE_IXXAT)
    list(APPEND UNICAN_SOURCES ixxat.cpp)
    add_compile_definitions(UNICAN_HAS_IXXAT)
endif()

# Create the main library target
# This creates the unican library with all configured sources
add_library(unican ${UNICAN_SOURCES})

# Set include directories
# Make the current directory available for #include statements
target_include_directories(unican PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Link against external libraries
# Find and link backend-specific libraries when backends are enabled
if(ENABLE_KVASER)
    find_library(KVASER_LIB canlib PATHS /usr/lib /usr/local/lib)
    if(KVASER_LIB)
        target_link_libraries(unican PRIVATE ${KVASER_LIB})
    else()
        message(WARNING "Kvaser CANlib not found")
    endif()
endif()

if(ENABLE_PCAN)
    find_library(PCAN_LIB pcanbasic PATHS /usr/lib /usr/local/lib)
    if(PCAN_LIB)
        target_link_libraries(unican PRIVATE ${PCAN_LIB})
    else()
        message(WARNING "PCAN Basic library not found")
    endif()
endif()

# Build example programs
# Create example executables when BUILD_EXAMPLES is enabled
if(BUILD_EXAMPLES)
    add_executable(simple_send_example examples/simple_send.cpp)
    target_link_libraries(simple_send_example unican)
endif()

# Installation rules
# Define what gets installed when 'make install' is run
install(TARGETS unican DESTINATION lib)
install(FILES 
    can_frame.hpp 
    can_bus.hpp 
    can_exception.hpp
    bus_socketcan.hpp
    DESTINATION include/unican
)