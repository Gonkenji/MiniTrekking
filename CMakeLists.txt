# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2_w CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(Trekking C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# 1. Procura e agrupa eventuais arquivos .c do driver ICM-20948 (se existirem na pasta)
file(GLOB ICM_DRIVER_SOURCES "lib/icm20948/*.c")

# 2. Adiciona o executável com todos os arquivos fonte do projeto (Sensores ToF, I2C, SPI/IMU)
add_executable(Trekking 
    src/Trekking_Core0.cpp
    src/Trekking_Core1.cpp
    lib/I2C.cpp
    lib/VL53L0X.cpp
    lib/VL53L1X.cpp
    lib/sensor_TOF.cpp
    lib/sensor_cor.cpp
    lib/navegacao.cpp
    lib/wifi.cpp
    lib/ICM20948_DMA.cpp
    ${ICM_DRIVER_SOURCES}
)

# 3. Define onde o compilador deve procurar os arquivos de cabeçalho (.h)
target_include_directories(Trekking PRIVATE 
    src
    lib
    ${CMAKE_CURRENT_LIST_DIR}
)

pico_set_program_name(Trekking "Trekking")
pico_set_program_version(Trekking "0.1")

# Generate PIO header
pico_generate_pio_header(Trekking ${CMAKE_CURRENT_LIST_DIR}/lib/encoder.pio)

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(Trekking 0)
pico_enable_stdio_usb(Trekking 1)

# Add all user requested libraries (I2C e SPI ativados)
target_link_libraries(Trekking
    pico_stdlib
    hardware_spi
    hardware_i2c
    hardware_pio
    hardware_pwm
    hardware_timer
    hardware_dma
    pico_multicore
    pico_cyw43_arch_lwip_threadsafe_background
)

pico_add_extra_outputs(Trekking)