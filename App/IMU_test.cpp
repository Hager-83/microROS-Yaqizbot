#include <stdio.h>
#include <string.h>


/* micro-ROS */
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rcl/error_handling.h>

/* ROS msgs */
#include "sensor_msgs/msg/imu.h"
#include <std_msgs/msg/float32.h>

/* Pico */
#include "pico/stdlib.h"
#include "pico/stdio_usb.h"
//#include "pico_usb_transport.h"
#include "../pico_uart_transports.h"


/* Services */
#include "Service/IMU/MPU9250_Service.hpp"

/* Publishers */
rcl_publisher_t imu_pub;

/* Messages */
sensor_msgs__msg__Imu imu_msg;

/* Timers */
rcl_timer_t imu_timer;
rcl_timer_t control_timer;

IMUService* imu_ptr = nullptr;


MPU9250_HAL imu_hal(i2c_default, MPU6500_DEFAULT_ADDRESS);
IMUService  imu_service(imu_hal);

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
    IMUData data = imu_ptr->getAll();
    
    imu_msg.linear_acceleration.x = data.accel.x_g * 9.81f;
    imu_msg.linear_acceleration.y = data.accel.y_g * 9.81f;
    imu_msg.linear_acceleration.z = data.accel.z_g * 9.81f;

    imu_msg.angular_velocity.x = data.gyro.x_dps * (3.14159f / 180.0f);
    imu_msg.angular_velocity.y = data.gyro.y_dps * (3.14159f / 180.0f);
    imu_msg.angular_velocity.z = data.gyro.z_dps * (3.14159f / 180.0f);

    rcl_ret_t ret = rcl_publish(&imu_pub, &imu_msg, NULL);
    if (ret != RCL_RET_OK) 
    {
        printf("Failed to publish IMU message\n");
    }

}

int main()
{
    // ---------- ADD: init stdio over USB ----------
    stdio_init_all();
    sleep_ms(2000); 
    printf("MAIN STARTED\n");
    // ---------- ADD: micro-ROS USB transport ----------
    rmw_uros_set_custom_transport(
    true,
    NULL,
    pico_serial_transport_open,
    pico_serial_transport_close,
    pico_serial_transport_write,
    pico_serial_transport_read
    );


    // Initialize IMU HAL (low-level sensor) - retry until successful


    while (!imu_service.IMUInit(PICO_DEFAULT_I2C_SDA_PIN,
                          PICO_DEFAULT_I2C_SCL_PIN,
                          400000))
    {
        printf("IMU SERVICE retry...\n");
        sleep_ms(500);
    }

    imu_ptr = &imu_service;

    printf("IMU Initialized successfully\n");


    // -------------------- micro-ROS variables --------------------
    rcl_timer_t timer;
    rcl_node_t node;
    rcl_allocator_t allocator;
    rclc_support_t support;
    rclc_executor_t executor;

    allocator = rcl_get_default_allocator();

    // Wait for agent successful ping for 2 minutes.
    const int timeout_ms = 1000; 
    const uint8_t attempts = 120;

    rcl_ret_t ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ret != RCL_RET_OK)
    {
        // Unreachable agent, exiting program.
        printf("Cannot reach micro-ROS agent --> exiting\n");
        return ret;
    }

    // -------------------- micro-ROS support & node --------------------
    rclc_support_init(&support, 0, NULL, &allocator);
    rclc_node_init_default(&node, "pico_node", "", &support);

    // -------------------- Publisher setup --------------------
    rclc_publisher_init_default(
        &imu_pub, 
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
        "imu_data"
    );

    // -------------------- Timer setup --------------------

    rcl_ret_t rc = rclc_timer_init_default2(
        &timer,
        &support,
        RCL_MS_TO_NS(1000),
        timer_callback,
        true  // use internal thread flag
    );
    if (rc != RCL_RET_OK) {
    printf("Timer init failed: %d\n", rc);
    }

    // -------------------- Executor setup --------------------
    rclc_executor_init(&executor, &support.context, 1, &allocator);
    rclc_executor_add_timer(&executor, &timer);

    while (true)
    {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
    }
    return 0;
}
