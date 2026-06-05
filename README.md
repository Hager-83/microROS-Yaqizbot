# Robot Control Node – Usage & Test Guide

This document explains **what the Robot Control Node does**, **how to build and run it**, and **how to test and verify that everything works as expected**.

---

## 1. Overview

The **Robot Control Node** runs on the Pico using **micro-ROS** and is responsible for:

* Receiving motion commands from ROS (`/cmd/vel`)
* Controlling **left & right motors** using **PID speed control**
* Reading wheel speed from **encoders**
* Publishing wheel RPM feedback
* Publishing **IMU data** (acceleration + angular velocity)

This node **does NOT do navigation or localization**.
It only executes **low-level control**.

---

## 2. Architecture (Big Picture)

```
ROS 2 (PC)
   |
   |  /cmd/vel  (geometry_msgs/Twist)
   v
Robot Control Node (Pico)
   |
   |-- Encoder HAL  ---> actual wheel RPM
   |-- PID          ---> throttle [-1 .. 1]
   |-- Motor HAL    ---> PWM + direction
   |
   |-- IMU Service  ---> accel + gyro
   |
   |--> /wheel/left/rpm   (Float32)
   |--> /wheel/right/rpm  (Float32)
   |--> /imu/data         (sensor_msgs/Imu)
```

---

## 3. Topics Summary

### Subscribed Topics

| Topic      | Type                  | Description                             |
| ---------- | --------------------- | --------------------------------------- |
| `/cmd/vel` | `geometry_msgs/Twist` | Desired robot linear & angular velocity |

### Published Topics

| Topic              | Type               | Description              |
| ------------------ | ------------------ | ------------------------ |
| `/wheel/left/rpm`  | `std_msgs/Float32` | Measured left wheel RPM  |
| `/wheel/right/rpm` | `std_msgs/Float32` | Measured right wheel RPM |
| `/imu/data`        | `sensor_msgs/Imu`  | Acceleration + gyro data |

---

## 4. Control Logic (What Happens Internally)

### 4.1 cmd_vel → Wheel RPM

* `linear.x` → forward/backward speed (m/s)
* `angular.z` → rotation around Z (rad/s)

Differential drive equations:

```
left_speed  = linear - (angular * wheel_base / 2)
right_speed = linear + (angular * wheel_base / 2)
```

Convert linear speed → RPM:

```
RPM = (speed / (2πR)) * 60
```

---

### 4.2 PID Speed Control

For each wheel:

```
error = target_rpm - measured_rpm
PID → throttle (-1 .. 1)
throttle → motor PWM + direction
```

* PID input = **encoder RPM**
* PID output = **motor throttle**
* IMU is **NOT used** in PID

---

### 4.3 IMU Publishing

* Runs on a **separate timer**
* Reads accel + gyro from MPU9250
* Publishes raw IMU data to ROS

No feedback from IMU to motors at this stage.

---

## 5. Timers & Callbacks

| Callback                 | Trigger        | Purpose                 |
| ------------------------ | -------------- | ----------------------- |
| `cmd_vel_callback`       | New `/cmd/vel` | Update target wheel RPM |
| `control_timer_callback` | 100 Hz         | Encoder → PID → Motor   |
| `imu_timer_callback`     | 50 Hz          | Read & publish IMU      |

Rule of thumb:

* **Callbacks** → react to external events
* **Timers** → run control loops

---

## 6. Build & Flash

### 6.1 Build

```bash
mkdir build
cd build
cmake ..
make
```

### 6.2 Flash to Pico

```bash
picotool load robot_control.uf2
```

(or via BOOTSEL + drag & drop)

---

## 7. Run micro-ROS Agent (PC)

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

Expected output:

```
[INFO] Agent running
[INFO] Client connected
```

---

## 8. Runtime Test Procedure

### 8.1 Check Node

```bash
ros2 node list
```

Expected:

```
/robot_control_node
```

---

### 8.2 Check Topics

```bash
ros2 topic list
```

Expected:

```
/cmd/vel
/wheel/left/rpm
/wheel/right/rpm
/imu/data
```

---

### 8.3 Send Motion Command

```bash
ros2 topic pub /cmd/vel geometry_msgs/Twist \
"{linear: {x: 0.2}, angular: {z: 0.0}}"
```

Expected behavior:

* Robot moves forward
* RPM topics update

---

### 8.4 Rotate in Place

```bash
ros2 topic pub /cmd/vel geometry_msgs/Twist \
"{linear: {x: 0.0}, angular: {z: 0.5}}"
```

Expected behavior:

* Left & right wheels rotate opposite directions

---

### 8.5 Monitor Feedback

```bash
ros2 topic echo /wheel/left/rpm
ros2 topic echo /wheel/right/rpm
ros2 topic echo /imu/data
```

---

## 9. Expected Console Output (Pico)

```
Robot node ready - waiting for /cmd_vel.
```

-------------------
-------------------


et me read everything fresh and carefully.Now I have a complete, verified picture of everything. Here is the full review:

---

## Full Review — Both Codebases (Last Version)

A lot of the previous issues have been fixed correctly. What remains are a critical bug, two functional bugs, and several smaller issues.

---

### 🔴 Critical — `imu_pub` is never registered

`imu_pub` is declared globally, `imu_timer_callback` publishes through it, but `rclc_publisher_init_default` is **never called for it** in `init_ros()`. The `joint_state_pub` init is there — the `imu_pub` init was accidentally removed.

Publishing through an uninitialised publisher handle is undefined behaviour on the Pico and will silently fail or crash.

```cpp
// Add this in init_ros(), alongside joint_state_pub init:
rclc_publisher_init_default(&imu_pub, &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu), "imu/data");
```

---

### 🔴 Critical — `spin_some` timeout kills your timer rates

Your timers are set to fire at **10ms (IMU)** and **50ms (control)**, but `spin_some` is given a **150ms** timeout:

```cpp
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(150));  // blocks 150ms each call
```

`spin_some` blocks for up to 150ms before returning, so the executor only gets to check for due timers **~6 times per second** — regardless of what period you set on the timers. Your IMU that you want at 100Hz is actually firing at ~6Hz, and your JointState that should be 20Hz is also throttled to ~6Hz. The TF graph confirms this: the working session shows **20.2Hz** which is what you'd expect from a 50ms timer only if spin is fast enough. Fix:

```cpp
// spin_some timeout must be ≤ shortest timer period
rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
```

---

### 🟡 Warning — Both timestamps hardcoded to zero

```cpp
joint_state_msg.header.stamp.sec     = 0;
joint_state_msg.header.stamp.nanosec = 0;
// same for imu_msg
```

This was a known placeholder. The TF graph from the working session shows timestamps around `1780616814` (Unix epoch), meaning the system is running with real time but zero-stamped messages from the Pico. `robot_localization` and any EKF will either reject these or behave incorrectly. The fix requires syncing the Pico clock with the agent:

```cpp
// In init_ros(), after rmw_uros_ping_agent succeeds:
rmw_uros_sync_session(1000);

// Then in each callback:
int64_t now_ms = rmw_uros_epoch_millis();
joint_state_msg.header.stamp.sec     = (int32_t)(now_ms / 1000);
joint_state_msg.header.stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000ULL);
```

---

### 🟡 Warning — `encoder_config.hpp` comments are wrong for every encoder

The comments say one thing, the pin numbers say another. The code uses `ENCODER1` for `enc_FL`, but the comment says `Encoder 1 → Motor RL`. This means either the comments are wrong or the physical wiring is swapped — and because the comments contradict themselves within the same file, it's impossible to know which is correct without checking the physical board.

```cpp
/* ── Encoder 1 — Motor RL (front-left) ───────────────────────── */  // ← says RL but calls it front-left
#define ENCODER1_PIN_A 20

/* ── Encoder 2 — Motor FL (rear-left) ────────────────────────── */  // ← says FL but calls it rear-left
#define ENCODER2_PIN_A 18
```

Every single encoder comment has the motor label and position label disagreeing. These need to be verified physically and corrected so the code is trustworthy. The actual behaviour depends entirely on whether the wiring matches pins 20/21 or 18/19 for FL — which only a physical check can confirm.

---

### 🟡 Warning — `v_fr` and `v_rr` negated without explanation

```cpp
double v_fr = - msg.velocity[2];   //<-----------------
double v_rr = - msg.velocity[3];
```

The comment marker `<---` suggests this was noticed but not explained. This negation is **correct only if** the right-side encoders are physically mounted in the mirrored direction (i.e. forward robot motion makes them c
Looking at this screenshot carefully — the RViz error **"No transform from [fl_wheel] to [odom]"** is still showing. This is the exact same screenshot as before (same timestamp 2:30 PM, same ROS Time 1780659050). Let me diagnose exactly why this is happening with the current code.Found it. The RViz error has one specific root cause that your TF frame files prove conclusively.

---

## The actual root cause — timestamps

Look at your own TF frame file from the working session:

```
odom → base_footprint:  Most recent = 1780616814.026  ← Unix time (real clock)
base_link → fl_wheel:   Most recent = 204.636          ← Pico uptime in seconds
```

The wheel TF is timestamped at **204 seconds** (3 minutes since Pico boot). The odom TF is at **1,780,616,814 seconds** (Unix epoch). RViz sees the wheel transform as **56 years old** relative to the odom frame and rejects it completely. That is why you get "No transform from [fl_wheel] to [odom]" — the transform exists, but RViz discards it as stale.

This is exactly the hardcoded zero timestamps in your code:

```cpp
// Current code — WRONG:
joint_state_msg.header.stamp.sec     = 0;
joint_state_msg.header.stamp.nanosec = 0;
```

`robot_state_publisher` forwards whatever timestamp is in the `JointState` message directly into the wheel TF. Zero or Pico-uptime timestamps go in, garbage timestamps come out.

---

## The fix — two lines in `Robot_control.cpp`

**Step 1 — sync the Pico clock once at startup, in `init_ros()`:**

```cpp
if (rmw_uros_ping_agent(1000, 120) != RCL_RET_OK)
{
    printf("No micro-ROS agent\n");
    return;
}

// ADD THIS — syncs Pico clock to agent's Unix time
rmw_uros_sync_session(1000);
```

**Step 2 — use the synced clock in `control_timer_callback`:**

```cpp
// REPLACE the hardcoded zeros with:
int64_t now_ms = rmw_uros_epoch_millis();
joint_state_msg.header.stamp.sec     = (int32_t)(now_ms / 1000);
joint_state_msg.header.stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000ULL);
```

**Step 3 — same in `imu_timer_callback`:**

```cpp
int64_t now_ms = rmw_uros_epoch_millis();
imu_msg.header.stamp.sec     = (int32_t)(now_ms / 1000);
imu_msg.header.stamp.nanosec = (uint32_t)((now_ms % 1000) * 1000000ULL);
```

---

## What happens after this fix

```
Before:  wheel TF stamp = 0s      vs  odom TF stamp = 1780616814s  → 56yr gap → RViz rejects
After:   wheel TF stamp = 1780...  vs  odom TF stamp = 1780...      → <1ms gap → RViz accepts
```

The wheels will appear on the robot body in RViz and rotate as the encoders move. This is the only remaining thing blocking the full model rendering — everything else (joint names, message type, topic name) is already correct.