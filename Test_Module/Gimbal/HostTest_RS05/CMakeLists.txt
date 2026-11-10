# RS05 版云台（Gimbal_test_RS05）的主机测试：在电脑上编译运行，不是 ARM 工程，不要加进 Gimbal_test_RS05 的 CMake。
# 场景和 ../HostTest（GM6020 版）是同一份 test_gimbal.c、假 main.h 也共用；这里只有电机模型（sim/）和假 can_bus（stub/）是 RS05 的。
# 用法见同目录 README.md。

cmake_minimum_required(VERSION 3.16)
project(gimbal_rs05_host_test C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../../..)
set(GIMBAL_APP ${CMAKE_CURRENT_LIST_DIR}/../Gimbal_test_RS05/App)
set(HOST_TEST_6020 ${CMAKE_CURRENT_LIST_DIR}/../HostTest)
set(SHARED_DIR ${REPO_ROOT}/Shared)

add_executable(gimbal_rs05_host_test
    ${HOST_TEST_6020}/test_gimbal.c
    sim/gimbal_sim.c
    stub/can_bus_stub.c

    # 被测的真代码
    ${GIMBAL_APP}/gimbal/gimbal.c
    ${GIMBAL_APP}/test_cmd/gimbal_test_cmd.c
    ${SHARED_DIR}/Algorithm/PID.c
    ${SHARED_DIR}/Algorithm/user_math.c
    ${SHARED_DIR}/Devices/RS05/rs05.c
    ${SHARED_DIR}/Link/chassis_link.c
)

# 假 main.h 用 GM6020 版的（../HostTest/stub），要排在最前面：被测代码 include 的 "main.h" 要找到假的那个。
# gimbal_sim.h 用本目录的：../HostTest/sim 不能加进来。
target_include_directories(gimbal_rs05_host_test PRIVATE
    ${HOST_TEST_6020}/stub
    sim
    ${GIMBAL_APP}/gimbal
    ${GIMBAL_APP}/test_cmd
    ${GIMBAL_APP}/ins
    ${SHARED_DIR}/Algorithm
    ${SHARED_DIR}/BSP
    ${SHARED_DIR}/Devices/RS05
    ${SHARED_DIR}/Link
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(gimbal_rs05_host_test PRIVATE -Wall -Wextra)
endif()

if(NOT MSVC)
    target_link_libraries(gimbal_rs05_host_test PRIVATE m)
endif()

enable_testing()
add_test(NAME gimbal_rs05_host_test COMMAND gimbal_rs05_host_test)
