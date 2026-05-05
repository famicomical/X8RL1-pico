cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(x8rl1_pico C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(x8rl1_pico
    src/main.c
    src/cz8rl1.c
    src/sampler.c
    src/timing.c
    src/usb_stream.c
)

target_include_directories(x8rl1_pico PRIVATE src)

target_link_libraries(x8rl1_pico
    pico_stdlib
    hardware_timer
    hardware_gpio
    hardware_irq
)

pico_enable_stdio_usb(x8rl1_pico 1)
pico_enable_stdio_uart(x8rl1_pico 0)

pico_add_extra_outputs(x8rl1_pico)
