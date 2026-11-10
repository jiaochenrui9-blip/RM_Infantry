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
set(CMAKE_PROJECT_NAME Gimbal_test_RS05)

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

# 各工程共用的 BSP / Devices / Algorithm / Link 放在 RM_Infantry/Shared，不在本工程目录内
get_filename_component(SHARED_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../../../Shared" ABSOLUTE)
if(NOT EXISTS "${SHARED_DIR}")
    message(FATAL_ERROR "Shared code directory not found: ${SHARED_DIR}")
endif()

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # 本工程的应用层：装配层 + 各模块
    App/robot.c
    App/gimbal/gimbal.c
    App/ins/ins.c
    App/test_cmd/gimbal_test_cmd.c

    # 云台电机是 RS05（和 Gimbal_test 的区别）。云台板上没有大疆电机，DJI_Driver 不编
    ${SHARED_DIR}/Devices/RS05/rs05.c
    ${SHARED_DIR}/Algorithm/PID.c
    ${SHARED_DIR}/Algorithm/user_math.c
    ${SHARED_DIR}/Algorithm/Attitude.c
    ${SHARED_DIR}/Devices/BMI088/BMI088.c
    ${SHARED_DIR}/Devices/IST8310/IST8310.c
    ${SHARED_DIR}/BSP/can_bus.c
    # 板间通信，和底盘板共用一份：云台板只用 ChassisLink_SendCmd
    ${SHARED_DIR}/Link/chassis_link.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    App
    App/gimbal
    App/ins
    App/test_cmd

    ${SHARED_DIR}/Devices/RS05
    ${SHARED_DIR}/Devices/BMI088
    ${SHARED_DIR}/Devices/IST8310
    ${SHARED_DIR}/Algorithm
    ${SHARED_DIR}/BSP
    ${SHARED_DIR}/Link
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
