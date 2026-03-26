/**
 * @file mbot_classic_ros.h
 * @brief MicroROS integration for MBot Omni
 */
#ifndef MBOT_CLASSIC_ROS_H
#define MBOT_CLASSIC_ROS_H

#include <stdio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <mbot/defs/mbot_params.h>
#include "config/mbot_classic_config.h"
#include <hardware/sync.h>

// Critical section macros
#define ENTER_CRITICAL() uint32_t __irq = save_and_disable_interrupts()
#define EXIT_CRITICAL() restore_interrupts(__irq)

// Drive mode definitions
enum drive_modes
{
    MODE_MOTOR_PWM = 0,      // Direct PWM control
    MODE_MOTOR_VEL = 1,      // motor velocity control
    MODE_MBOT_VEL = 2        // Robot body velocity control
};

// Robot state structure
typedef struct {
    int64_t timestamp_us;
    // Odometry
    float odom_x;
    float odom_y;
    float odom_theta;
    // Body velocity
    float vx;
    float vy;
    float wz;
    // Wheel state
    float wheel_vel[NUM_MOT_SLOTS];
    int32_t encoder_ticks[NUM_MOT_SLOTS];
    int32_t encoder_delta_ticks[NUM_MOT_SLOTS];
    // Motor control
    float motor_pwm[NUM_MOT_SLOTS];
    // IMU data
    float imu_gyro[3];  // x, y, z
    float imu_accel[3]; // x, y, z
    float imu_quat[4];  // w, x, y, z
    float imu_mag[3];   // Magnetometer data
    float imu_rpy[3];   // Roll, pitch, yaw
    // Analog inputs
    float analog_in[4]; // 4 ADC channels
    // Encoder data
    int64_t last_encoder_time;
    int64_t encoder_delta_t;
} mbot_state_t;

// Command structure
typedef struct {
    int64_t timestamp_us;   // pico time
    float vx;
    float vy;
    float wz;
    float wheel_vel[NUM_MOT_SLOTS]; // 0=left, 1=right, 2=back
    float motor_pwm[NUM_MOT_SLOTS]; // 0=left, 1=right, 2=back
    int drive_mode;  // 0=PWM, 1=wheel vel, 2=body vel
} mbot_cmd_t;

// Extern declarations for global state variables defined in mbot_classic_ros.c
extern mbot_state_t mbot_state;
extern mbot_cmd_t mbot_cmd;

int mbot_init_micro_ros(void);
int mbot_spin_micro_ros(void);
int mbot_init_micro_ros_comm(void);

#endif /* MBOT_CLASSIC_ROS_H */