# 云台主机测试：在电脑上编译运行，不是 ARM 工程，不要加进 Gimbal_test 的 CMake。
# 用法见同目录 README.md。

cmake_minimum_required(VERSION 3.16)
project(gimbal_host_test C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
set(GIMBAL_APP ${CMAKE_CURRENT_LIST_DIR}/../Gimbal_test/App)
set(SHARED_DIR ${REPO_ROOT}/Shared)

add_executable(gimbal_host_test
    test_gimbal.c
    sim/gimbal_sim.c
    stub/can_bus_stub.c

    # 被测的真代码
    ${GIMBAL_APP}/gimbal/gimbal.c
    ${GIMBAL_APP}/test_cmd/gimbal_test_cmd.c
    ${SHARED_DIR}/Algorithm/PID.c
    ${SHARED_DIR}/Algorithm/user_math.c
    ${SHARED_DIR}/Devices/DJI_Driver/dji_motor.c
    ${SHARED_DIR}/Devices/DJI_Driver/dji_motor_manager.c
    ${SHARED_DIR}/Devices/DJI_Driver/dji_gm6020.c
    ${SHARED_DIR}/Devices/DJI_Driver/motor_control.c
)

# stub 放最前面：被测代码 include 的 "main.h" 要找到假的那个
target_include_directories(gimbal_host_test PRIVATE
    stub
    sim
    ${GIMBAL_APP}/gimbal
    ${GIMBAL_APP}/test_cmd
    ${GIMBAL_APP}/ins
    ${SHARED_DIR}/Algorithm
    ${SHARED_DIR}/BSP
    ${SHARED_DIR}/Devices/DJI_Driver
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(gimbal_host_test PRIVATE -Wall -Wextra)
endif()

if(NOT MSVC)
    target_link_libraries(gimbal_host_test PRIVATE m)
endif()

enable_testing()
add_test(NAME gimbal_host_test COMMAND gimbal_host_test)
