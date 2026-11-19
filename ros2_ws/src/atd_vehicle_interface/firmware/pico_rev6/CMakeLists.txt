cmake_minimum_required(VERSION 3.17)

if(NOT PICO_SDK_PATH AND DEFINED ENV{PICO_SDK_PATH})
  set(PICO_SDK_PATH "$ENV{PICO_SDK_PATH}")
endif()
if(NOT PICO_SDK_PATH)
  get_filename_component(PICO_SDK_PATH "${CMAKE_CURRENT_LIST_DIR}/.cache/pico-sdk" ABSOLUTE)
endif()
set(PICO_BOARD pico CACHE STRING "Raspberry Pi Pico RP2040")
set(PICO_NO_PICOTOOL 1 CACHE BOOL "UF2 is generated and verified by scripts/uf2.py")
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")

project(jpbb_pico_rev6 C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(TINYUSB_OPT_OS OPT_OS_NONE)
pico_sdk_init()

if(NOT PICO_RP2040 OR NOT PICO_BOARD STREQUAL "pico")
  message(FATAL_ERROR "This firmware is for the original RP2040 Raspberry Pi Pico on JPBB Rev6")
endif()
if(NOT PICO_SDK_VERSION_STRING VERSION_EQUAL "2.3.1")
  message(FATAL_ERROR "Use the pinned Pico SDK 2.3.1; run scripts/bootstrap.sh")
endif()

add_executable(jpbb_pico_rev6
  src/main.c
  src/rev6_hal.c
  src/rev6_core.c
  src/rev6_led.c
  src/rev6_sbus.c
  src/rev6_transport.c
  src/usb_descriptors.c
  src/jpbb_protocol.c
)
target_include_directories(jpbb_pico_rev6 PRIVATE
  "${CMAKE_CURRENT_LIST_DIR}/include"
)
target_compile_options(jpbb_pico_rev6 PRIVATE -Wall -Wextra)
target_compile_definitions(jpbb_pico_rev6 PRIVATE
  PICO_STDIO_USB=0
  PICO_STDIO_UART=0
  PICO_STDIO_USB_ENABLE_RESET_VIA_BAUD_RATE=0
  PICO_STDIO_USB_ENABLE_RESET_VIA_VENDOR_INTERFACE=0
)
target_link_libraries(jpbb_pico_rev6 PRIVATE
  pico_stdlib pico_unique_id pico_bootrom
  hardware_adc hardware_clocks hardware_gpio hardware_irq hardware_pwm
  hardware_sync hardware_uart hardware_watchdog
  tinyusb_device tinyusb_board
)
pico_enable_stdio_usb(jpbb_pico_rev6 0)
pico_enable_stdio_uart(jpbb_pico_rev6 0)
pico_set_program_name(jpbb_pico_rev6 "JPBB Rev6 Pico")
pico_set_program_version(jpbb_pico_rev6 "0.2.0-led-bench")
pico_add_extra_outputs(jpbb_pico_rev6)
