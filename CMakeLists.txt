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
set(CMAKE_PROJECT_NAME Hero-Gimbal)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        Task/Src/motorTask.cpp
        Task/Src/masterBoardTask.cpp
        Task/Inc/imuTask.h
        Task/Src/imuTask.cpp
        Task/Src/usbTask.cpp
        Task/Inc/usbTask.h
        BMI088/Src/BMI088.cpp
        BMI088/Src/ImuTempControl.cpp
        BMI088/BMI088config.h
        Bsp/Src/bsp_can.cpp
        Bsp/Src/bsp_dwt.cpp
        Bsp/magictools.h
        PID/Src/angle_pid.cpp
        PID/Src/speed_pid.cpp
        PID/Src/temp_pid.cpp
        Debug-Vars/debug_vars.cpp
        Message_Bus/message_bus.cpp
        Control/Src/chassis_ctrl.cpp
        Control/Inc/chassis_ctrl.h
        Control/chassis_cmd.h
        Control/chassis_strategy.h
        Control/Src/chassis_stop.cpp
        Control/Inc/chassis_stop.h
        Control/Src/chassis_followgimbal.cpp
        Control/Inc/chassis_followgimbal.h
        Control/Src/chassis_spin.cpp
        Control/Inc/chassis_spin.h
        Algorithm/Src/MahonyAHRS.c
        Algorithm/Src/CRC16.cpp
        Algorithm/Src/CRC8.cpp
        Algorithm/Inc/CRC16.h
        Algorithm/Inc/CRC8.h
        Algorithm/Src/FusionAHRS.cpp
        Algorithm/Inc/FusionAHRS.h
        Communication/Src/usart_protocol.cpp
        Communication/Src/usart_dma.cpp
        Communication/Src/usart_decode.cpp
        Communication/Src/dbus.cpp
        Communication/Src/hc05_gamepad.cpp
        Communication/Src/usb_decode.cpp
        Communication/Inc/usb_decode.h
        Communication/Src/usb_protocol.cpp
        Communication/Inc/usb_protocol.h
        Communication/Src/referee_protocol.cpp
        Communication/Inc/referee_protocol.h
        Communication/Src/referee_decode.cpp
        Communication/Inc/referee_decode.h
        RobotArm/Src/arm_math.cpp
        RobotArm/Inc/arm_math.h
        RobotArm/Src/kinematics.cpp
        RobotArm/Inc/kinematics.h
        RobotArm/Src/trajectory.cpp
        RobotArm/Inc/trajectory.h
        RobotArm/Src/arm_hal_dm.cpp
        RobotArm/Inc/arm_hal.h
        RobotArm/Src/arm_controller.cpp
        RobotArm/Inc/arm_controller.h
        RobotArm/Src/arm_protocol.cpp
        RobotArm/Inc/arm_protocol.h
        RobotArm/Inc/arm_types.h
        RobotArm/Inc/arm_config.h
        Task/Src/armTask.cpp
        Task/Inc/armTask.h
        Task/Src/armDebugTask.cpp
        Task/Inc/armDebugTask.h)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined include paths
        Algorithm/Inc
        BMI088/Inc
        Bsp/Inc
        Bsp
        BMI088
        PID
        Task/Inc
        PID/Inc
        Debug-Vars
        Communication/Inc
        Message_Bus
        Control
        Control/Inc
        RobotArm/Inc
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
        # Add user defined symbols
)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
        stm32cubemx

        # Add user defined libraries
)
