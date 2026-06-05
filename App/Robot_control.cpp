#include <stdio.h>
#include <math.h>  // if needed in future calculations

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <rcl/error_handling.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include "sensor_msgs/msg/imu.h"

#include <sensor_msgs/msg/joint_state.h>

/* Pico */
#include "pico/stdlib.h"
extern "C" {
    #include "pico_uart_transports.h"
}
#include "Robot_control.hpp"
#include "MPU9250_Service.hpp"

#include "encoder_config.hpp"                  
#include "encoder_hal.hpp"
#include "encoder_service.hpp"

#include "motor_config.hpp"
#include "motor_hal.hpp"
#include "motor_service.hpp"  



// micro-ROS objects 
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

// micro-ROS objects 
/* Publishers */
rcl_publisher_t imu_pub;
sensor_msgs__msg__Imu imu_msg; 

rcl_publisher_t joint_state_pub;
sensor_msgs__msg__JointState joint_state_msg;

// Static arrays 
static double js_positions[4];
static double js_velocities[4];
static double js_effort[4] = {0.0, 0.0, 0.0, 0.0};

static const char* joint_names[4] = {
    "fl_wheel_joint",
    "rl_wheel_joint",
    "fr_wheel_joint",
    "rr_wheel_joint"
};

IMUService* imu_ptr = nullptr;

MPU9250_HAL imu_hal(i2c_default, MPU6500_DEFAULT_ADDRESS);
IMUService  imu_service(imu_hal);

// Encoder objects
EncoderHAL enc_FL(ENCODER1_PIN_A, ENCODER1_PIN_B);   // front-left
EncoderHAL enc_RL(ENCODER2_PIN_A, ENCODER2_PIN_B);   // rear-left
EncoderHAL enc_FR(ENCODER3_PIN_A, ENCODER3_PIN_B);   // front-right
EncoderHAL enc_RR(ENCODER4_PIN_A, ENCODER4_PIN_B);   // rear-right


EncoderService svc_enc_FL(enc_FL);
EncoderService svc_enc_RL(enc_RL);
EncoderService svc_enc_FR(enc_FR);
EncoderService svc_enc_RR(enc_RR);


// Motor objects 
MotorHal motor_FL(MOTOR_FL_IN1, MOTOR_FL_IN2, MOTOR_FL_EN);
MotorHal motor_RL(MOTOR_RL_IN1, MOTOR_RL_IN2, MOTOR_RL_EN);
MotorHal motor_FR(MOTOR_FR_IN1, MOTOR_FR_IN2, MOTOR_FR_EN);
MotorHal motor_RR(MOTOR_RR_IN1, MOTOR_RR_IN2, MOTOR_RR_EN);

MotorService svc_motor_FL(motor_FL);
MotorService svc_motor_RL(motor_RL);
MotorService svc_motor_FR(motor_FR);
MotorService svc_motor_RR(motor_RR);


rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rcl_timer_t control_timer, imu_timer;
rclc_executor_t executor;

// Kinematics parameters (based on robot) 
const float WHEEL_BASE_M  = 0.28f;    // distance between left and right wheels in metres (measure and confirm)
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

    // right side motor are physically reversed 
    svc_motor_FR.setTargetSpeed(-right_target); //<----------------
    svc_motor_RR.setTargetSpeed(-right_target);
    

    // Debug print
    //printf("cmd_vel: lin=%.2f ang=%.2f → L=%.2f R=%.2f\n",linear, angular, left_target, right_target);
}

// ────────────────────────────────────────────────
// Timer callback - reads values and publishes them
// ────────────────────────────────────────────────
void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    (void) last_call_time;
    if (timer == NULL) return;

    static const double WHEEL_RADIUS_M = WHEEL_RADIUS_CM / 100.0;

    // Positions (rad)
    js_positions[0] = svc_enc_FL.encoderGetPositionRad();
    js_positions[1] = svc_enc_RL.encoderGetPositionRad();
    js_positions[2] = svc_enc_FR.encoderGetPositionRad();
    js_positions[3] = svc_enc_RR.encoderGetPositionRad();

    // Velocities: cm/s → rad/s
    js_velocities[0] = (svc_enc_FL.encoderGetSpeedCmS() / 100.0f) / WHEEL_RADIUS_M;
    js_velocities[1] = (svc_enc_RL.encoderGetSpeedCmS() / 100.0f) / WHEEL_RADIUS_M;
    js_velocities[2] = (svc_enc_FR.encoderGetSpeedCmS() / 100.0f) / WHEEL_RADIUS_M;
    js_velocities[3] = (svc_enc_RR.encoderGetSpeedCmS() / 100.0f) / WHEEL_RADIUS_M;

    // Timestamp
    // In init_ros(), after rmw_uros_ping_agent succeeds:
    rmw_uros_sync_session(1000);

    // Then in each callback:
    int64_t now_ms = rmw_uros_epoch_millis();
    joint_state_msg.header.stamp.sec     = (int32_t)(now_ms / 1000);
    joint_state_msg.header.stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000ULL);

    joint_state_msg.position.data  = js_positions;
    joint_state_msg.velocity.data  = js_velocities;
    joint_state_msg.effort.data    = js_effort;
    joint_state_msg.position.size  = 4;
    joint_state_msg.velocity.size  = 4;
    joint_state_msg.effort.size    = 4;

    rcl_ret_t ret = rcl_publish(&joint_state_pub, &joint_state_msg, NULL);
    if (ret != RCL_RET_OK)
        printf("Failed to publish JointState\n");
}
 
void imu_timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    IMUData data = imu_ptr->getAll();
    

    imu_msg.linear_acceleration.x = data.accel.x_g * 9.80665f;
    imu_msg.linear_acceleration.y = data.accel.y_g * 9.80665f;
    imu_msg.linear_acceleration.z = data.accel.z_g * 9.80665f;

    imu_msg.angular_velocity.x = data.gyro.x_dps * (M_PI / 180.0f);
    imu_msg.angular_velocity.y = data.gyro.y_dps * (M_PI / 180.0f);
    imu_msg.angular_velocity.z = data.gyro.z_dps * (M_PI / 180.0f);

    // Timestamp
    int64_t now_ms = rmw_uros_epoch_millis();
    imu_msg.header.stamp.sec     = (int32_t)(now_ms / 1000);
    imu_msg.header.stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000ULL);


    rcl_ret_t ret = rcl_publish(&imu_pub, &imu_msg, NULL);
    if (ret != RCL_RET_OK) 
    {
        printf("Failed to publish IMU message\n");
    }

}
void RobotSystem::init()
{
    stdio_init_all();
    sleep_ms(2000);
    printf("Robot System Init\n");

    init_transport();
    init_hardware();
    init_ros();
}

void RobotSystem::init_transport()
{
    rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
}
void RobotSystem::init_hardware()
{
    // IMU  
    while (!imu_service.IMUInit(PICO_DEFAULT_I2C_SDA_PIN,
                               PICO_DEFAULT_I2C_SCL_PIN,
                               400000))
    {
        printf("IMU retry...\n");
        sleep_ms(500);
    }

    imu_ptr = &imu_service;

    // Encoders  
    svc_enc_FL.encoderInit();
    svc_enc_RL.encoderInit();
    svc_enc_FR.encoderInit();
    svc_enc_RR.encoderInit();

    // Motors
    svc_motor_FL.motorInit();
    svc_motor_RL.motorInit();
    svc_motor_FR.motorInit();
    svc_motor_RR.motorInit();
    
    svc_enc_FL.encoderStart();
    svc_enc_RL.encoderStart();
    svc_enc_FR.encoderStart();
    svc_enc_RR.encoderStart();
}
 
void RobotSystem::init_ros()
{
    allocator = rcl_get_default_allocator();

    if (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK)
    {
        printf("No micro-ROS agent\n");
        return;
    }

    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "robot_control_node", "", &support);


    // Subscriber
    rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "cmd_vel"
    );

    // Publishers
    rclc_publisher_init_default(&joint_state_pub, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState), "joint_states");

    // Add this in init_ros(), alongside joint_state_pub init:
    rclc_publisher_init_default(&imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");

    sensor_msgs__msg__JointState__init(&joint_state_msg);

    joint_state_msg.header.frame_id.data     = (char*)"base_link";
    joint_state_msg.header.frame_id.size     = strlen("base_link");
    joint_state_msg.header.frame_id.capacity = strlen("base_link") + 1;

    joint_state_msg.name.data = (rosidl_runtime_c__String*)
    allocator.allocate(4 * sizeof(rosidl_runtime_c__String), allocator.state);
    joint_state_msg.name.size     = 4;
    joint_state_msg.name.capacity = 4;

    for (int i = 0; i < 4; i++) {
        joint_state_msg.name.data[i].data     = (char*)joint_names[i];
        joint_state_msg.name.data[i].size     = strlen(joint_names[i]);
        joint_state_msg.name.data[i].capacity = strlen(joint_names[i]) + 1;
    }

    joint_state_msg.position.data     = js_positions;
    joint_state_msg.position.size     = 4;
    joint_state_msg.position.capacity = 4;

    joint_state_msg.velocity.data     = js_velocities;
    joint_state_msg.velocity.size     = 4;
    joint_state_msg.velocity.capacity = 4;

    joint_state_msg.effort.data     = js_effort;
    joint_state_msg.effort.size     = 4;
    joint_state_msg.effort.capacity = 4;


    // Timers
    rclc_timer_init_default2(&control_timer, &support,
        RCL_MS_TO_NS(50), control_timer_callback, true);

    rclc_timer_init_default2(&imu_timer, &support,
        RCL_MS_TO_NS(10), imu_timer_callback, true);

    // Executor
    rclc_executor_init(&executor, &support.context, 3, &allocator);

    rclc_executor_add_subscription(&executor,
        &cmd_vel_sub, &cmd_vel_msg,
        &cmd_vel_callback, ON_NEW_DATA);

    rclc_executor_add_timer(&executor, &control_timer);
    rclc_executor_add_timer(&executor, &imu_timer);
}

void RobotSystem::spin()
{
    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
    }
}


