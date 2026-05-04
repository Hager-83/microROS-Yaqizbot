#include <stdio.h>

#include <rcl/rcl.h>
#include <rclc/rclc.h>

#include <rclc/executor.h>
#include <rcl/error_handling.h>

#include <rmw_microros/rmw_microros.h>

#include "pico/stdlib.h"
#include "../pico_uart_transports.h"   

#include <geometry_msgs/msg/twist.h>
#include "sensor_msgs/msg/imu.h"
#include <std_msgs/msg/float32.h>


#include "MPU9250_Service.hpp"

#include "motor_config.hpp"
#include "motor_hal.hpp"
#include "motor_service.hpp"  

#include "encoder_config.hpp"                  
#include "encoder_hal.hpp"
#include "encoder_service.hpp"


// micro-ROS objects 
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

// micro-ROS objects 
/* Publishers */
rcl_publisher_t imu_pub;

rcl_publisher_t left_speed_pub;
rcl_publisher_t right_speed_pub;

rcl_publisher_t left_rpm_pub;
rcl_publisher_t right_rpm_pub;


/* Messages */
sensor_msgs__msg__Imu imu_msg;

std_msgs__msg__Float32 left_speed_msg;
std_msgs__msg__Float32 right_speed_msg;

std_msgs__msg__Float32 left_rpm_msg;
std_msgs__msg__Float32 right_rpm_msg;


IMUService* imu_ptr = nullptr;

// Encoder objects

EncoderHAL enc_FL(ENCODER1_PIN_A, ENCODER1_PIN_B);   // front-left
EncoderHAL enc_RL(ENCODER2_PIN_A, ENCODER2_PIN_B);   // rear-left
EncoderHAL enc_FR(ENCODER3_PIN_A, ENCODER3_PIN_B);   // front-right
EncoderHAL enc_RR(ENCODER4_PIN_A, ENCODER4_PIN_B);   // rear-right


EncoderService svc_enc_FL(enc_FL);
EncoderService svc_enc_RL(enc_RL);
EncoderService svc_enc_FR(enc_FR);
EncoderService svc_enc_RR(enc_RR);


// Motor objects (using pins from config)

MotorHal motor_FL(MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_EN);
MotorHal motor_RL(MOTOR_RL_IN1, MOTOR_RL_IN2, MOTOR_RL_EN);
MotorHal motor_FR(MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_EN);
MotorHal motor_RR(MOTOR_RR_IN1, MOTOR_RR_IN2, MOTOR_RR_EN);

MotorService svc_motor_FL(motor_FL);
MotorService svc_motor_RL(motor_RL);
MotorService svc_motor_FR(motor_FR);
MotorService svc_motor_RR(motor_RR);


// Kinematics parameters (based on robot) 
const float WHEEL_BASE_M  = 0.28f;    // Assume distance between wheels in meters (25 cm = 0.25 m)
const float MAX_SPEED_M_S = 1.0f;     // Maximum allowed speed in m/s

// cmd_vel callback: converts Twist to target speed for each motor
void cmd_vel_callback(const void *msgin) {
    const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;

    float linear  = msg->linear.x;   // Forward/backward speed in m/s
    float angular = msg->angular.z;  // Rotation speed in rad/s

    // Differential drive kinematics
    float left_target  = linear - (angular * WHEEL_BASE_M / 2.0f);
    float right_target = linear + (angular * WHEEL_BASE_M / 2.0f);

    // Normalize to -1.0 → +1.0 (relative to max speed)
    left_target  /= MAX_SPEED_M_S;
    right_target /= MAX_SPEED_M_S;

    // Send target speed to each motor
    svc_motor_FL.setTargetSpeed(left_target);
    svc_motor_RL.setTargetSpeed(left_target);

    svc_motor_FR.setTargetSpeed(right_target);
    svc_motor_RR.setTargetSpeed(right_target);

    // Debug print
    //printf("cmd_vel: lin=%.2f ang=%.2f → L=%.2f R=%.2f\n",linear, angular, left_target, right_target);
}

// ────────────────────────────────────────────────
// Timer callback - reads values and publishes them
// ────────────────────────────────────────────────
void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;  // suppress unused parameter warning

    if (timer == NULL)
    {
        return;
    }
    else{}

    // Read current values from encoder services
    float speed_FL      = svc_enc_FL.encoderGetSpeedCmS();
    float speed_RL      = svc_enc_RL.encoderGetSpeedCmS();

    float speed_FR      = svc_enc_FR.encoderGetSpeedCmS();
    float speed_RR      = svc_enc_RR.encoderGetSpeedCmS();

    float rpm_FL        = svc_enc_FL.encoderGetRPM();
    float rpm_RL        = svc_enc_RL.encoderGetRPM();

    float rpm_FR        = svc_enc_FR.encoderGetRPM();
    float rpm_RR        = svc_enc_RR.encoderGetRPM();
    

    // Fill messages
    left_speed_msg.data  = (speed_FL + speed_RL) / 2.0f;
    right_speed_msg.data = (speed_FR + speed_RR) / 2.0f;

    left_rpm_msg.data    = (rpm_FL + rpm_RL) / 2.0f;
    right_rpm_msg.data   = (rpm_FR + rpm_RR) / 2.0f;

    // Publish all
    rcl_ret_t ret;

    ret = rcl_publish(&left_speed_pub, &left_speed_msg, NULL);
    if (ret != RCL_RET_OK)
    {
        printf("Failed to publish left speed. \n");
    }

    ret = rcl_publish(&right_speed_pub, &right_speed_msg, NULL);
    if (ret != RCL_RET_OK)
    {
        printf("Failed to publish right speed. \n");
    }

    ret = rcl_publish(&left_rpm_pub, &left_rpm_msg, NULL);
    if (ret != RCL_RET_OK)
    {
        printf("Failed to publish left RPM. \n");
    }

    ret = rcl_publish(&right_rpm_pub, &right_rpm_msg, NULL);
    if (ret != RCL_RET_OK)
    {
        printf("Failed to publish right RPM.\n");
    }

}

void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    IMUData data = imu_ptr->getAll();
    

    imu_msg.linear_acceleration.x = data.accel.x_g;
    imu_msg.linear_acceleration.y = data.accel.y_g;
    imu_msg.linear_acceleration.z = data.accel.z_g;

    imu_msg.angular_velocity.x = data.gyro.x_dps;
    imu_msg.angular_velocity.y = data.gyro.y_dps;
    imu_msg.angular_velocity.z = data.gyro.z_dps;

    rcl_ret_t ret = rcl_publish(&imu_pub, &imu_msg, NULL);
    if (ret != RCL_RET_OK) 
    {
        printf("Failed to publish IMU message\n");
    }

}


int main() {
    // Initialize standard I/O
    stdio_init_all();
    sleep_ms(2000);
    printf("Pico micro-ROS Motor Node started\n");

    // Set custom serial transport for micro-ROS
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );


    // Initialize the IMU
    MPU9250_HAL imu_hal(i2c_default, MPU6500_DEFAULT_ADDRESS);
    IMUService imu_service(imu_hal);
    imu_ptr = &imu_service;

    // Initialize IMU HAL (low-level sensor) - retry until successful
    while (!imu_hal.begin(PICO_DEFAULT_I2C_SDA_PIN, PICO_DEFAULT_I2C_SCL_PIN, 400000))
    {
        printf("IMU HAL begin failed, retrying...\n");
        sleep_ms(500);
    }

    while (!imu_service.begin())
    {
        printf("IMU Service begin failed, retrying...\n");
        sleep_ms(500);
    }


    // Initialize encoders and start periodic calculation (100 ms timer inside service)
    svc_enc_FL.encoderInit();
    svc_enc_RL.encoderInit();

    svc_enc_FR.encoderInit();
    svc_enc_RR.encoderInit();

    svc_enc_FL.encoderStart();
    svc_enc_RL.encoderStart();

    svc_enc_FR.encoderStart();
    svc_enc_RR.encoderStart();

    // Initialize motors 
    svc_motor_FL.motorInit();
    svc_motor_RL.motorInit();
    svc_motor_FR.motorInit();
    svc_motor_RR.motorInit();

    // micro-ROS core setup
    rcl_allocator_t allocator = rcl_get_default_allocator();
    rclc_support_t support;
    rcl_node_t node;
    rcl_timer_t control_timer, imu_timer;
    rclc_executor_t executor;

    // Wait for micro-ROS agent (up to 120 seconds)
    const int timeout_ms = 1000;
    const uint8_t attempts = 120;
    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        printf("Cannot reach micro-ROS agent → exiting\n");
        return ret;
    }

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "motors_node", "", &support);

    // Subscriber: /cmd_vel
    rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    // Create publishers

    rclc_publisher_init_default(
        &imu_pub, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_data"
    );

    rclc_publisher_init_default(
        &left_speed_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "left_wheel_speed"
    );

    rclc_publisher_init_default(
        &right_speed_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "right_wheel_speed"
    );

    rclc_publisher_init_default(
        &left_rpm_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "left_wheel_rpm"
    );

    rclc_publisher_init_default(
        &right_rpm_pub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32),
        "right_wheel_rpm"
    );

    // Create timer (publish rate)
    const unsigned int timer_period_ms = 150;   // 150 ms = ~6.67 Hz

    rclc_timer_init_default(
        &control_timer,
        &support,
        RCL_MS_TO_NS(timer_period_ms),
        control_timer_callback
    );

    rclc_timer_init_default(
        &imu_timer, 
        &support,
        RCL_MS_TO_NS(20), 
        imu_timer_callback
    );

    // Executor: handles subscription and timer
    rclc_executor_init(&executor, &support.context, 3, &allocator); // 1 sub + 2 timer + margin
    rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA);
    rclc_executor_add_timer(&executor, &control_timer);
    rclc_executor_add_timer(&executor, &imu_timer);

    //printf("Motor node ready - waiting for /cmd_vel\n");

    while (true) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }

    // Cleanup (rarely reached)
    rclc_executor_fini(&executor);
    rcl_node_fini(&node);
    rclc_support_fini(&support);

    return 0;
}