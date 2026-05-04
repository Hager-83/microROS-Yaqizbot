cmake_minimum_required(VERSION 3.12)

# Import Pico SDK 
include(pico_sdk_import.cmake)
#include($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(IMU_test C CXX ASM)  #<-------------------------
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize Pico SDK
pico_sdk_init()

# Add the micro-ROS agent lib directory
link_directories(libmicroros)

# Add your executable with micro-ROS example source + IMU HAL and Service sources
add_executable(robot_control_node   #<-------------------------
    App/IMU_test.cpp                     # main micro-ROS app  #<-------------------------
    pico_uart_transport.c                # micro-ROS serial transport

    # MPU9250 files (HAL + Service):
    HAL/IMU/MPU9250_HAL.cpp          
    Service/IMU/MPU9250_Service.cpp  

    # Encoder files (HAL + Service):
    HAL/Encoder/encoder_hal.cpp
    Service/Encoder/encoder_service.cpp

    # Motor files (HAL + Service):
    HAL/Motor/motor_hal.cpp
    Service/Motor/motor_service.cpp

    # PID files (Service):
    Service/PID/PID.cpp

)

# Link libraries
target_link_libraries(robot_control_node #<-------------------------
    pico_stdlib
    microros
    hardware_i2c                   # add hardware i2c for MPU9250 communication
    hardware_gpio                  # add gpio library (needed for sensors and LEDs)
    hardware_pwm                   # for motor PWM
    hardware_timer                 # for repeating_timer in encoder_service.cpp
)

target_include_directories(robot_control_node PUBLIC #<-------------------------
    libmicroros/include

    # IMU
    HAL/IMU                       # IMU hal headers path
    Service/IMU                   # IMU service headers path
    
    # Encoder
    HAL/Encoder                   # Encoder hal headers path
    Service/Encoder               # Encoder service headers path

    # Motor
    HAL/Motor                     # Motor hal headers path
    Service/Motor                 # Motor service headers path

    # PID
    Service/PID                   # PID service headers path
    .

)


# Compiler flags for size optimization (keep as is)
SET(CMAKE_C_FLAGS  "${CMAKE_C_FLAGS} -ffunction-sections -fdata-sections")
SET(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -ffunction-sections -fdata-sections")

# Configure Pico 
pico_enable_stdio_usb(robot_control_node 1)   # Enable USB stdio for serial output   #<-------------------------
pico_enable_stdio_uart(robot_control_node 0)  # Disable UART stdio to avoid conflict #<-------------------------

add_compile_definitions(PICO_UART_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_ENABLE_CRLF_SUPPORT=0)
add_compile_definitions(PICO_STDIO_DEFAULT_CRLF=0)

# Generate UF2 output for flashing
pico_add_extra_outputs(robot_control_node)  #<-------------------------
