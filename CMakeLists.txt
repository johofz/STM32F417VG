# Template CMakeLists.txt configuration for STM32 MCUS
cmake_minimum_required(VERSION 3.12)

set(CMAKE_TOOLCHAIN_FILE gcc-arm-none-eabi.cmake)

# ##############################################################################
# Edit your MCU information up to the next border
project(STM32F417VG)

set(MCU_FAMILY STM32F4xx)
set(MCU_MODEL STM32F417xx)
set(CPU_PARAMETERS
    -mcpu=cortex-m4
    -mthumb
    -mfpu=fpv4-sp-d16
    -mfloat-abi=hard)

# ##############################################################################
set(LIBRARY ${PROJECT_NAME})
enable_language(C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS ON)

# Headers
set(INCLUDE_DIRECTORIES
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/CMSIS/Include
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/ST/Include)

# Sources
file(GLOB_RECURSE SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.c
    ${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/Drivers/ST/src/*.c)

# Executable files
add_library(${LIBRARY}
    ${SOURCES}
)

# Embedded macros(defines)
target_compile_definitions(${LIBRARY} PRIVATE
    ${MCU_MODEL})

# Add header directories (AFTER add_executable !!)
target_include_directories(${LIBRARY} PRIVATE
    ${INCLUDE_DIRECTORIES})

# Compiler and linker options
target_compile_options(${LIBRARY} PRIVATE
    ${CPU_PARAMETERS}
    -Wall
    -Wextra
    -Wpedantic
    -Wno-missing-field-initializers
    -Wno-unused-parameter
    $<$<CONFIG:Debug>:-Og -g3 -ggdb>
    $<$<CONFIG:Release>:-Og -g0>)