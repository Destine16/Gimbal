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
    Modules/bsp/bsp_dwt.c
    Modules/message_center/message_center.c
    Modules/daemon/daemon.c
    Modules/algorithm/controller.c
    Modules/algorithm/crc16.c
    Modules/algorithm/kalman_filter.c
    Modules/algorithm/QuaternionEKF.c
    Modules/motor/gm6020.c
    Modules/imu/BMI088Middleware.c
    Modules/imu/BMI088driver.c
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
    Modules
    Modules/bsp
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
)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx
    /home/skyvector/basic_framework/Middlewares/ST/ARM/DSP/Lib/libarm_cortexM4lf_math.a

    # Add user defined libraries
)
