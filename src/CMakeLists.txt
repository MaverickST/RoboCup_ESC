cmake_minimum_required(VERSION 3.13)

set(PICO_BOARD "pico_w")

# initialize the SDK based on PICO_SDK_PATH
# note: this must happen before project()
include(pico_sdk_import.cmake)


project(ESC_RoboCup)

# initialize the Raspberry Pi Pico SDK
pico_sdk_init()

add_executable(esc_robocup
	main.c
)

target_include_directories(esc_robocup PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

# Add pico_stdlib library which aggregates commonly used features
target_link_libraries(esc_robocup 
	pico_stdlib
	pico_flash
	pico_time
	hardware_timer
	pico_cyw43_arch_none
	hardware_gpio
	hardware_irq 
	hardware_sync
	hardware_i2c
	hardware_pwm)

pico_enable_stdio_uart(esc_robocup 0)
pico_enable_stdio_usb(esc_robocup 1)

# create map/bin/hex/uf2 file in addition to ELF.
pico_add_extra_outputs(esc_robocup)

# Memory usage
SET(GCC_EXE_LINKER_FLAGS    "-Wl,--print-memory-usage")

SET(CMAKE_EXE_LINKER_FLAGS  "${CMAKE_EXE_LINKER_FLAGS} ${GCC_EXE_LINKER_FLAGS}")
