cmake_minimum_required(VERSION 3.12)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(pico_sdk_import.cmake)
project(robomagellan_firmware)
pico_sdk_init()

set(WIZNET_CHIP W5500)
add_definitions(-DWIZCHIP_=5500)
set(WIZNET_DIR ${CMAKE_SOURCE_DIR}/libraries/ioLibrary_Driver)
set(PORT_DIR ${CMAKE_SOURCE_DIR}/libraries/port)

add_subdirectory(${CMAKE_SOURCE_DIR}/libraries)
add_subdirectory(${CMAKE_SOURCE_DIR}/libraries/port)

add_executable(robomagellan_firmware
  main.c
)
target_link_libraries(robomagellan_firmware
  pico_stdlib
  hardware_adc
  hardware_clocks
  hardware_dma
  hardware_pwm
  hardware_spi
  ETHERNET_FILES
  IOLIBRARY_FILES
  TIMER_FILES
)

pico_enable_stdio_usb(robomagellan_firmware 1)
pico_add_extra_outputs(robomagellan_firmware)
