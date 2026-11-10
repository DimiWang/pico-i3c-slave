cmake_minimum_required(VERSION 3.13)

# Pico SDK: set PICO_SDK_PATH, or pass -DPICO_SDK_FETCH_FROM_GIT=ON to let
# pico_sdk_import.cmake download it.
include(pico_sdk_import.cmake)

project(pico_i3c_slave C CXX ASM)
set(CMAKE_C_STANDARD 11)

pico_sdk_init()

add_executable(pico_i3c_slave
    src/main.c
    src/i3c_phy.c
    src/i3c_target.c
    src/i3c_log.c
)

pico_generate_pio_header(pico_i3c_slave ${CMAKE_CURRENT_LIST_DIR}/src/i3c_target.pio)

target_include_directories(pico_i3c_slave PRIVATE ${CMAKE_CURRENT_LIST_DIR}/src)

# Optional overrides: cmake -DI3C_STATIC_ADDR=0x50 -DI3C_SDA_PIN=14 ...
foreach(opt I3C_SDA_PIN I3C_SCL_PIN I3C_PID I3C_BCR I3C_DCR I3C_STATIC_ADDR
            I3C_RESPOND_ON_STATIC_ADDR I3C_LEGACY_I2C I3C_SYS_CLK_KHZ
            I3C_MWL I3C_MRL)
    if(DEFINED ${opt})
        target_compile_definitions(pico_i3c_slave PRIVATE ${opt}=${${opt}})
    endif()
endforeach()

target_compile_definitions(pico_i3c_slave PRIVATE
    PICO_BUILD=1
    PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0
    # Same VID:PID as every pico-sdk CDC device (2E8A:000A) - including the
    # I3CBlaster itself - so give the port a distinct product string.
    USBD_PRODUCT="pico-i3c-slave"
)
target_compile_options(pico_i3c_slave PRIVATE -Wall -Wextra -Wno-unused-parameter -O2)

target_link_libraries(pico_i3c_slave
    pico_stdlib
    pico_multicore
    hardware_pio
    hardware_clocks
    hardware_vreg
)

# Run everything from RAM: the bus core must never stall on XIP flash.
pico_set_binary_type(pico_i3c_slave copy_to_ram)

pico_enable_stdio_usb(pico_i3c_slave 1)
pico_enable_stdio_uart(pico_i3c_slave 0)

pico_set_program_name(pico_i3c_slave "pico-i3c-slave")
pico_set_program_description(pico_i3c_slave "I3C SDR target for testing I3CBlaster")
pico_set_program_url(pico_i3c_slave "https://github.com/DimiWang/pico-i3c-slave")

pico_add_extra_outputs(pico_i3c_slave)
