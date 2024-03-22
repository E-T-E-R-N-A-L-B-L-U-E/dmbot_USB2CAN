cmake_minimum_required(VERSION 3.15)
project(dmbot_usb2can)

set(CMAKE_CXX_STANDARD 17)

include_directories(
    includes
)

add_library(serial_lib
    STATIC
    src/serial/serial.cc
    src/serial/impl/unix.cc
    src/serial/impl/list_ports/list_ports_linux.cc
)
target_link_directories(serial_lib PUBLIC includes/serial)

add_library(usb2can_lib
    STATIC
    src/can/can_bus.cpp
)
target_link_directories(usb2can_lib PUBLIC includes/can)
target_link_libraries(usb2can_lib serial_lib)


add_executable(can_bus_demo app/can_bus_demo.cpp)
target_link_libraries(can_bus_demo usb2can_lib serial_lib)