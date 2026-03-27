# MBot Omni Firmware (ROS2 Jazzy)
- Hardware: Pico 2040 + Pi5, 3-wheel omnidirectional drive (120-degree spacing)
- Software: Ubuntu 24 + ROS2 Jazzy + Microros

## How to use
```bash
# clone the repository
cd ~
gh repo clone mbot-project/mbot_firmware_ros
cd ~/mbot_firmware_ros

# compile the firmware code
mkdir build
cd build
cmake ..
make
```
Then flash the firmware to the pico board:
```bash
cd ~/mbot_firmware_ros/build
# calibration - wait until the robot stops moving
sudo mbot-upload-firmware flash mbot_calibrate_omni.uf2
# main firmware
sudo mbot-upload-firmware flash mbot_omni_ros.uf2
```
## Project Components
1. `libmicroros`: This directory contains the precompiled micro-ROS static library (`libmicroros.a`) and all necessary header files for the Raspberry Pi Pico. This library includes:
    - ROS 2 client library core functionality
    - Message type definitions
    - Transport layer implementations
    - Serialization/deserialization utilities
2. `microros_static_library`: Contains scripts and configuration files used to generate the `libmicroros` static library. We use it when we need to add customized ros data types. The key components include:
    - `library_generation.sh`: Script that sets up the build environment, compiles micro-ROS packages, and generates the static library
    - `colcon.meta`: Configuration for the colcon build system
    - `toolchain.cmake`: CMake toolchain file for cross-compiling to Raspberry Pi Pico
3. `comms`: Communication Setup
4. `mbot`: MBot Hardware Library
5. `rc` (Robot Control): A library providing essential functions for robot control applications.
6. Supporting Files
    - `available_ros2_types`: A list of all ROS 2 message, service, and action types available in the micro-ROS library
    - `built_packages`: A list of all Git repositories and their specific commit hashes used to build the micro-ROS library

## Overview

### Drive System
The MBot Omni uses three omnidirectional wheels at 120-degree spacing for holonomic (3-DOF) motion:
- **MOT_L** (slot 2): Left wheel
- **MOT_R** (slot 0): Right wheel
- **MOT_B** (slot 1): Back wheel

Physical parameters:
- Wheel radius: 0.048 m (96 mm diameter)
- Base radius: 0.10843 m (center to wheel)
- Encoder resolution: 20 counts/revolution
- Gear ratio: 78:1

### Kinematics
**Forward kinematics** (wheel velocities to body velocity):
```
vx = R * (wL / sqrt(3) - wR / sqrt(3))
vy = R * (-wL/3 - wR/3 + wB * 2/3)
wz = -R * (wL + wR + wB) / (3 * L)
```
**Inverse kinematics** (body velocity to wheel velocities):
```
wL = (sqrt(3)/2 * vx - vy/2 - L * wz) / R
wR = (-sqrt(3)/2 * vx - vy/2 - L * wz) / R
wB = (vy - L * wz) / R
```
Where R = wheel radius, L = base radius, wL/wR/wB = wheel angular velocities.

### Directory Structure
- `src/`: microROS-based implementation
- `include/config/mbot_classic_config.h`: MBot omni hardware config file
- `include/config/mbot_classic_default_pid.h`: Default PID gains for all 3 wheels
- `mbot/include/mbot/defs/mbot_params.h`: MBot system config and FRAM parameter structure

### Code Modules in `src/`
- `mbot_classic_ros.c/.h`: Core application logic, main loop, 3-motor hardware interface, omni kinematics, and microROS node initialization.
- `mbot_ros_comms.c/.h`: Handles all ROS-specific communication aspects including publisher/subscriber setup, message initialization, and callbacks.
- `mbot_odometry.c/.h`: Odometry calculation utilities (supports full holonomic motion with vx, vy, wz).
- `mbot_controller.c/.h`: PID velocity controllers for all 3 wheel motors, with ROS parameter server for live tuning.
- `mbot_print.c/.h`: Debug printing utilities.

### Publishers
| Topic        | Pub Rate | QoS         |
| -------------- | ---------- | :------------ |
| /battery_adc | 4 HZ    | Best Effort |
| /encoders    | 50 HZ    | Best Effort |
| /imu         | 100 HZ   | Best Effort |
| /motor_vel   | 100 HZ   | Best Effort |
| /odom        | 25 HZ    | Reliable    |
| /tf          | 50 HZ    | Reliable |

### MBot State
- Using local state variables for robot state (`mbot_state_t`, `mbot_cmd_t`)
- Synchronizing state between hardware readings and ROS messages (hardware readings update `mbot_state`, ROS messages are populated from `mbot_state`, `mbot_cmd` updated by ROS callbacks).
Key global state variables:
   - `mbot_state_t mbot_state`: Defined in `mbot_classic_ros.c`, holds the current snapshot of all robot sensor data (3 motors), odometry, and derived states. Updated by `mbot_loop`. Read by `mbot_publish_state`.
   - `mbot_cmd_t mbot_cmd`: Defined in `mbot_classic_ros.c`, stores the latest commands received via ROS subscriptions (supports vx, vy, wz for holonomic control). Updated by ROS callbacks in `mbot_ros_comms.c`. Read by motor control logic in `mbot_loop`.
   - `mbot_params_t params`: Stores calibration parameters loaded from FRAM (motor/encoder polarity, slope/intercept, and PID gains for all 3 wheels).

### PID Tuning
All 3 wheel PID controllers can be tuned live via the ROS 2 parameter server:
- `left_wheel.kp`, `left_wheel.ki`, `left_wheel.kd`, `left_wheel.tf`
- `right_wheel.kp`, `right_wheel.ki`, `right_wheel.kd`, `right_wheel.tf`
- `back_wheel.kp`, `back_wheel.ki`, `back_wheel.kd`, `back_wheel.tf`

Changes are automatically saved to FRAM for persistence across reboots.

## Communication
The firmware uses a single USB Type-C connection with dual CDC (Communication Device Class) interfaces:
1. **Debug Channel** (`/dev/mbot_debug`):
   - Used for firmware debug messages and status prints
   - Accessible via standard serial tools: `sudo minicom -D /dev/mbot_debug -b 115200`

2. **MicroROS Channel** (`/dev/mbot_microros`):
   - Dedicated to microROS communication
   - Handles all ROS2 messages and services
   - Used by micro-ros-agent for ROS2 bridge

This dual-channel approach allows simultaneous debugging and ROS communication without additional hardware connections.

### Important: Servicing TinyUSB and Using `sleep_ms`

**TinyUSB (the USB stack used for CDC communication) requires frequent servicing via `tud_task()` (`dual_cdc_task()`).**
If the USB stack is not serviced regularly (ideally every 1-10 ms), the host computer may think the device is unresponsive and disconnect the serial ports (`/dev/mbot_debug`, `/dev/mbot_microros`).

**Do NOT use long `sleep_ms()` calls.**

#### **Incorrect Usage (will cause USB ports to disappear):**
```c
// BAD: This will block USB for 2 seconds!
sleep_ms(2000);
```

#### **Correct Usage (keeps USB alive):**
Use the wait function we provide.
```c
#include "comms/dual_cdc.h";
mbot_wait_ms(2000);
```

## Time Management
- Using ROS time synchronization
- Maintaining fixed-rate control loops (e.g., 25Hz for sensor acquisition and control logic via `mbot_loop`)
- Periodic ROS state publishing (e.g., 20Hz via `timer_callback` triggered by `ros_publish_timer`)

Variables:
- `mbot_state.timestamp_us` is the last time the mbot_state was updated by `mbot_loop`.
- `mbot_state.last_encoder_time` is the Pico local time recorded at the start of the encoder read in `mbot_loop`, used for calculating `encoder_delta_t`.
- `now` variable (local to `mbot_publish_state` function) holds the ROS-synchronized epoch time obtained via `rmw_uros_epoch_nanos()`, used for timestamping outgoing ROS messages.

## Calibration
Run the calibration firmware (`mbot_calibrate_omni.uf2`) before first use. The calibration:
1. Tests all 8 motor polarity combinations (2^3 for 3 motors) using gyroscope feedback to find the correct CCW rotation
2. Determines motor and encoder polarity for all 3 wheels
3. Measures motor slope/intercept curves (positive and negative directions) for feedforward control
4. Stores default PID gains for all 3 wheels
5. Saves all parameters to FRAM for persistent storage

## Build Targets
| Target | Output | Description |
|--------|--------|-------------|
| `mbot_omni_ros` | `mbot_omni_ros.uf2` | Main firmware with ROS2 communication |
| `mbot_calibrate_omni` | `mbot_calibrate_omni.uf2` | Motor calibration firmware |
| `mbot_omni_motor_test` | `mbot_omni_motor_test.uf2` | Individual motor test firmware |
