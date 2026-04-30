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
set(CMAKE_PROJECT_NAME Gimbal)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

option(IMU_ONLY_BRINGUP_ENABLE "Bring up only IMU-related tasks and skip robot/motor init" OFF)
option(GIMBAL_YAW_ONLY_TEST_ENABLE "Enable temporary yaw-only gimbal test mode" OFF)
option(GIMBAL_PITCH_ONLY_TEST_ENABLE "Enable temporary pitch-only gimbal test mode" OFF)
option(VISION_DEBUG_RTT_ENABLE "Enable normal vision-control RTT debug telemetry" OFF)
option(IMU_MOUNT_CALIBRATION_ENABLE "Keep gimbal motors zero-force while collecting IMU mount calibration RTT data" OFF)
set(VISION_CONTROL_MODE "1" CACHE STRING "Vision control mode: 0=continuous delta, 1=event delta to held target")
set_property(CACHE VISION_CONTROL_MODE PROPERTY STRINGS 0 1)
set(GIMBAL_SYSID_MODE "0" CACHE STRING "Gimbal system identification mode: 0=off, 1=yaw PRBS, 2=pitch feedforward, 3=yaw step, 4=pitch hysteresis, 5=yaw perf step, 6=pitch perf step, 7=yaw perf sine, 8=pitch perf sine")
set_property(CACHE GIMBAL_SYSID_MODE PROPERTY STRINGS 0 1 2 3 4 5 6 7 8)

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

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    Application/robot.c
    Application/robot_task.c
    Application/cmd/robot_cmd.c
    Application/gimbal/gimbal.c
    Application/gimbal/gimbal_params.c
    Application/sysid/gimbal_sysid.c
    Application/sysid/gimbal_sysid_telemetry.c
    Modules/bsp/bsp_dwt.c
    Modules/debug/rtt_backend.c
    Modules/debug/sysid_rtt.c
    Modules/debug/vision_debug_rtt.c
    Modules/message_center/message_center.c
    Modules/daemon/daemon.c
    Modules/algorithm/controller.c
    Modules/algorithm/crc16.c
    Modules/algorithm/kalman_filter.c
    Modules/algorithm/QuaternionEKF.c
    Modules/motor/gm6020_device.c
    Modules/motor/gm6020_control.c
    Modules/motor/gm6020_can.c
    Modules/imu/BMI088Middleware.c
    Modules/imu/bmi088_core.c
    Modules/imu/bmi088_init.c
    Modules/imu/bmi088_async.c
    Modules/imu/ins_task.c
    Modules/vision/vision_comm.c
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    Application
    Application/cmd
    Application/gimbal
    Application/sysid
    Modules
    Modules/bsp
    Modules/debug
    Modules/message_center
    Modules/daemon
    Modules/algorithm
    Modules/motor
    Modules/imu
    Modules/vision
    /home/skyvector/basic_framework/Middlewares/ST/ARM/DSP/Include
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
    VISION_CONTROL_MODE=${VISION_CONTROL_MODE}
    GIMBAL_SYSID_MODE=${GIMBAL_SYSID_MODE}
)

if(VISION_DEBUG_RTT_ENABLE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE VISION_DEBUG_RTT_ENABLE=1)
else()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE VISION_DEBUG_RTT_ENABLE=0)
endif()

if(IMU_MOUNT_CALIBRATION_ENABLE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE IMU_MOUNT_CALIBRATION_ENABLE=1)
else()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE IMU_MOUNT_CALIBRATION_ENABLE=0)
endif()

if((NOT GIMBAL_SYSID_MODE STREQUAL "0") OR VISION_DEBUG_RTT_ENABLE)
    target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        /home/skyvector/basic_framework/Middlewares/Third_Party/SEGGER/RTT/SEGGER_RTT.c
    )
    target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        /home/skyvector/basic_framework/Middlewares/Third_Party/SEGGER/RTT
        /home/skyvector/basic_framework/Middlewares/Third_Party/SEGGER/Config
    )
endif()

if(IMU_ONLY_BRINGUP_ENABLE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE IMU_ONLY_BRINGUP_ENABLE=1)
else()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE IMU_ONLY_BRINGUP_ENABLE=0)
endif()

if(GIMBAL_YAW_ONLY_TEST_ENABLE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE GIMBAL_YAW_ONLY_TEST_ENABLE=1)
else()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE GIMBAL_YAW_ONLY_TEST_ENABLE=0)
endif()

if(GIMBAL_PITCH_ONLY_TEST_ENABLE)
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE GIMBAL_PITCH_ONLY_TEST_ENABLE=1)
else()
    target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE GIMBAL_PITCH_ONLY_TEST_ENABLE=0)
endif()

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    /home/skyvector/basic_framework/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a

    # Add user defined libraries
)
