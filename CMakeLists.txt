cmake_minimum_required(VERSION 3.12)

include(pico_sdk_import.cmake)
include(pico_extras_import_optional.cmake)

project(picoisp C CXX ASM)
set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(
    picoisp
	main.c
)


# pull in common dependencies and additional pwm hardware support
target_link_libraries(
	picoisp
	pico_stdlib
	pico_multicore
	hardware_clocks
	hardware_resets
	hardware_pwm
	pico_bootsel_via_double_reset
)

pico_enable_stdio_usb(picoisp 1)
pico_enable_stdio_uart(picoisp 0)

target_compile_definitions(
	picoisp PRIVATE
	PICO_BOOTSEL_VIA_DOUBLE_RESET_ACTIVITY_LED=25
)

# create map/bin/hex file etc.
pico_add_extra_outputs(picoisp)
