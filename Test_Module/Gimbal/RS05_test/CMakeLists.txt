cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME RS05_test)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME})

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# 各工程共用的 BSP / Devices / Algorithm 放在 RM_Infantry/Shared，不在本工程目录内
get_filename_component(SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Shared" ABSOLUTE)
if(NOT EXISTS "${SHARED_DIR}")
    message(FATAL_ERROR "Shared code directory not found: ${SHARED_DIR}")
endif()

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # 本工程只有一个文件：RS05 单电机测试（Robot_Init / Robot_Loop 和 CAN 接收回调）
    App/robot.c

    # 用到的公共代码：RS05 驱动（和 Gimbal_test_RS05 同一份）、CAN 收发、限幅
    ${SHARED_DIR}/Devices/RS05/rs05.c
    ${SHARED_DIR}/BSP/can_bus.c
    ${SHARED_DIR}/Algorithm/user_math.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App

    ${SHARED_DIR}/Devices/RS05
    ${SHARED_DIR}/BSP
    ${SHARED_DIR}/Algorithm
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)
