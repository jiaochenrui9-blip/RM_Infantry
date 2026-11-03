# 整车主机仿真：云台板 + 底盘板的真代码编在一起，在电脑上运行。不是 ARM 工程，不要加进任何板子的 CMake。
# 用法见同目录 README.md。

cmake_minimum_required(VERSION 3.16)
project(car_sim C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)

set(REPO_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)
set(SHARED_DIR ${REPO_ROOT}/Shared)
set(GIMBAL_APP ${REPO_ROOT}/Test_Module/Gimbal/Gimbal_test/App)
set(CHASSIS_APP ${REPO_ROOT}/Test_Module/Chassis/Chassis_test/App)
# 假的 main.h 和云台主机测试共用一份（HAL 类型、HAL_GetTick 声明、开关中断）
set(HOST_STUB ${REPO_ROOT}/Test_Module/Gimbal/HostTest/stub)

add_executable(car_sim
    test_car.c
    sim/car_sim.c
    stub/can_bus_stub.c

    # 被测的真代码：云台板
    ${GIMBAL_APP}/gimbal/gimbal.c
    ${GIMBAL_APP}/test_cmd/gimbal_test_cmd.c
    # 被测的真代码：底盘板
    ${CHASSIS_APP}/Infantry_Chassis.c
    # 被测的真代码：两块板共用
    ${SHARED_DIR}/Link/chassis_link.c
    ${SHARED_DIR}/Algorithm/PID.c
    ${SHARED_DIR}/Algorithm/user_math.c
    ${SHARED_DIR}/Devices/DJI_Driver/dji_motor.c
    ${SHARED_DIR}/Devices/DJI_Driver/dji_motor_manager.c
    ${SHARED_DIR}/Devices/DJI_Driver/dji_gm6020.c
    ${SHARED_DIR}/Devices/DJI_Driver/dji_m3508.c
    ${SHARED_DIR}/Devices/DJI_Driver/motor_control.c
)

# HOST_STUB 放最前面：被测代码 include 的 "main.h" 要找到假的那个
target_include_directories(car_sim PRIVATE
    ${HOST_STUB}
    sim
    ${GIMBAL_APP}/gimbal
    ${GIMBAL_APP}/test_cmd
    ${GIMBAL_APP}/ins
    ${CHASSIS_APP}
    ${SHARED_DIR}/Link
    ${SHARED_DIR}/Algorithm
    ${SHARED_DIR}/BSP
    ${SHARED_DIR}/Devices/DJI_Driver
)

if(CMAKE_C_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(car_sim PRIVATE -Wall -Wextra)
endif()

if(NOT MSVC)
    target_link_libraries(car_sim PRIVATE m)
endif()

enable_testing()
add_test(NAME car_sim COMMAND car_sim)
