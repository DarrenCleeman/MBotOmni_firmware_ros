/**
 * @file mbot_classic_ros.c
 * @brief MicroROS integration for MBot Omni (3-wheel omnidirectional)
 */
#include <math.h>
#include <rcl/error_handling.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>
#include <rmw_microros/time_sync.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <pico/multicore.h>
#include "pico/time.h"
#include <hardware/clocks.h>
#include <hardware/adc.h>

// mbotlib
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>
#include <mbot/fram/fram.h>
#include <mbot/imu/imu.h>
#include <mbot/utils/utils.h>

// mbot_classic_ros
#include "mbot_classic_ros.h"
#include "mbot_odometry.h"
#include "mbot_ros_comms.h"
#include "mbot_print.h"
#include "mbot_controller.h"

// comms
#include <comms/pico_uart_transports.h>
#include <comms/dual_cdc.h>

// robot control
#include <rc/math/filter.h>

#ifndef PI
#define PI 3.14159265358979323846f
#endif
const float adc_conversion_factor = 3.0f / (1 << 12);

// Global state variables
mbot_state_t mbot_state = {0};
mbot_cmd_t mbot_cmd = {0};
mbot_params_t params;
mbot_bhy_config_t mbot_imu_config;
mbot_bhy_data_t mbot_imu_data;
static bool enable_pwm_lpf = true;
rc_filter_t mbot_left_pwm_lpf;
rc_filter_t mbot_right_pwm_lpf;
rc_filter_t mbot_back_pwm_lpf;

// Global MicroROS objects
static rcl_allocator_t allocator;
static rclc_support_t support;
static rcl_node_t node;
static rclc_executor_t executor;
static rclc_parameter_server_t parameter_server;

// Timer for periodic publishing
static rcl_timer_t ros_publish_timer;
static repeating_timer_t mbot_loop_timer;

int mbot_init_micro_ros(void);
int mbot_spin_micro_ros(void);
static void mbot_publish_state(void);
static bool mbot_loop(repeating_timer_t *rt);
void timer_callback(rcl_timer_t * timer, int64_t last_call_time);

// helper functions
static int mbot_init_hardware(void);
static void mbot_read_imu(void);
static void mbot_read_encoders(void);
static void mbot_read_adc(void);
static float calibrated_pwm_from_vel_cmd(float vel_cmd, int motor_idx);
static void mbot_calculate_motor_vel(void);
static void mbot_calculate_omni_body_vel(float wheel_left_vel, float wheel_right_vel, float wheel_back_vel, float* vx, float* vy, float* wz);
static void print_mbot_params(const mbot_params_t* params);
static void core1_usb_task(void);

// Thread-safe helpers for mbot_state and mbot_cmd
static void get_mbot_state_safe(mbot_state_t* dest);
static void set_mbot_state_safe(const mbot_state_t* src);
static void get_mbot_cmd_safe(mbot_cmd_t* dest);
static void set_mbot_cmd_safe(const mbot_cmd_t* src);

// Initialize microROS
int mbot_init_micro_ros(void) {
    allocator = rcl_get_default_allocator();

    rcl_ret_t ret = rclc_support_init(&support, 0, NULL, &allocator);
    if (ret != RCL_RET_OK) {
        printf("rclc_support_init failed: %d\n", ret);
        return MBOT_ERROR;
    }

    // Synchronize time with agent
    rmw_ret_t sync_ret = rmw_uros_sync_session(2000);
    if (sync_ret != RMW_RET_OK) {
        printf("[FATAL] Time sync with agent failed: %d\n", sync_ret);
        while(1) { tight_loop_contents(); }
    }

    ret = rclc_node_init_default(&node, "mbot_control_node", "", &support);
    if (ret != RCL_RET_OK) {
        printf("rclc_node_init_default failed: %d\n", ret);
        rclc_support_fini(&support);
        return MBOT_ERROR;
    }

    ret = mbot_ros_comms_init_messages(&allocator);
    if (ret != MBOT_OK) return MBOT_ERROR;

    ret = mbot_ros_comms_init_publishers(&node);
    if (ret != MBOT_OK) return MBOT_ERROR;

    ret = mbot_ros_comms_init_subscribers(&node);
    if (ret != MBOT_OK) return MBOT_ERROR;

    ret = mbot_ros_comms_init_services(&node);
    if (ret != MBOT_OK) return MBOT_ERROR;

    // Initialize parameter server
    ret = init_parameter_server(&parameter_server, &node);
    if (ret != MBOT_OK) return MBOT_ERROR;

    ret = rclc_timer_init_default(
        &ros_publish_timer,
        &support,
        RCL_MS_TO_NS((int)(ROS_TIMER_PERIOD * 1000)),
        timer_callback);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Timer init failed: %d\n", ret);
        return MBOT_ERROR;
    }

    // Initialize executor with enough handles for parameter server, subscribers, timer, and services
    ret = rclc_executor_init(&executor, &support.context, RCLC_EXECUTOR_PARAMETER_SERVER_HANDLES + 6, &allocator);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Failed to init executor: %d\n", ret);
        return MBOT_ERROR;
    }

    ret = rclc_executor_add_timer(&executor, &ros_publish_timer);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Adding timer to executor failed: %d\n", ret);
        return MBOT_ERROR;
    }

    ret = rcl_timer_call(&ros_publish_timer);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Timer call failed: %d\n", ret);
    }

    // Add parameter server to executor
    ret = rclc_executor_add_parameter_server(&executor, &parameter_server, parameter_callback);
    if (ret != RCL_RET_OK) {
        printf("[ERROR] Failed to add parameter server to executor: %d\n", ret);
        return MBOT_ERROR;
    }

    ret = mbot_ros_comms_add_to_executor(&executor);
    if (ret != MBOT_OK) return MBOT_ERROR;

    return MBOT_OK;
}

// Handle incoming ROS messages
int mbot_spin_micro_ros(void) {
    rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
    if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
        printf("microROS spin error: %d\r\n", ret);
        return MBOT_ERROR;
    }

    return MBOT_OK;
}

// Publish all robot state to ROS topics
static void mbot_publish_state(void) {
    static uint8_t tick = 0;
    tick++;

    rcl_ret_t ret;

    // IMU - every callback
    ret = rcl_publish(&imu_publisher, &imu_msg, NULL);

    // Motor velocities - every callback
    mbot_state_t local_state;
    get_mbot_state_safe(&local_state);
    motor_vel_msg.velocity[MOT_L] = local_state.wheel_vel[MOT_L];
    motor_vel_msg.velocity[MOT_R] = local_state.wheel_vel[MOT_R];
    motor_vel_msg.velocity[MOT_B] = local_state.wheel_vel[MOT_B];
    ret = rcl_publish(&motor_vel_publisher, &motor_vel_msg, NULL);

    // Encoders & TF - every 2nd tick (50 Hz at 100 Hz timer)
    if (tick % 2 == 0) {
        ret = rcl_publish(&encoders_publisher, &encoders_msg, NULL);
        ret = rcl_publish(&tf_publisher, &tf_msg, NULL);
    }

    // Odometry - every 4th tick (25 Hz at 100 Hz timer)
    if (tick % 4 == 0) {
        ret = rcl_publish(&odom_publisher, &odom_msg, NULL);
    }

    // Battery - every 25th tick (4 Hz at 100 Hz timer)
    if (tick % 25 == 0) {
        ret = rcl_publish(&battery_publisher, &battery_msg, NULL);
    }
}

// Main robot logic loop, runs at MAIN_LOOP_HZ (called by hardware timer)
static bool mbot_loop(repeating_timer_t *rt) {
    {   // Critical section for sensor and state update
        ENTER_CRITICAL();
        mbot_read_encoders();
        mbot_read_imu();
        mbot_read_adc();
        mbot_calculate_motor_vel();
        mbot_calculate_omni_body_vel(
            mbot_state.wheel_vel[MOT_L],
            mbot_state.wheel_vel[MOT_R],
            mbot_state.wheel_vel[MOT_B],
            &mbot_state.vx,
            &mbot_state.vy,
            &mbot_state.wz
        );
        mbot_calculate_odometry(
            mbot_state.vx,
            mbot_state.vy,
            mbot_state.wz,
            MAIN_LOOP_PERIOD,
            &mbot_state.odom_x,
            &mbot_state.odom_y,
            &mbot_state.odom_theta
        );
        /* Populate odometry and TF ROS messages with the exact timestamp of this calculation */
        int64_t stamp_ns = rmw_uros_epoch_nanos();

        /* Odometry message */
        odom_msg.header.stamp.sec     = stamp_ns / 1000000000LL;
        odom_msg.header.stamp.nanosec = stamp_ns % 1000000000LL;
        odom_msg.pose.pose.position.x = mbot_state.odom_x;
        odom_msg.pose.pose.position.y = mbot_state.odom_y;
        odom_msg.pose.pose.position.z = 0.0f;

        float cy = cosf(mbot_state.odom_theta * 0.5f);
        float sy = sinf(mbot_state.odom_theta * 0.5f);
        odom_msg.pose.pose.orientation.w = cy;
        odom_msg.pose.pose.orientation.x = 0.0f;
        odom_msg.pose.pose.orientation.y = 0.0f;
        odom_msg.pose.pose.orientation.z = sy;

        odom_msg.twist.twist.linear.x  = mbot_state.vx;
        odom_msg.twist.twist.linear.y  = mbot_state.vy;
        odom_msg.twist.twist.angular.z = mbot_state.wz;

        /* TF message (odom -> base_footprint) */
        tf_msg.transforms.data[0].header.stamp.sec     = odom_msg.header.stamp.sec;
        tf_msg.transforms.data[0].header.stamp.nanosec = odom_msg.header.stamp.nanosec;
        tf_msg.transforms.data[0].header.frame_id      = odom_msg.header.frame_id;
        tf_msg.transforms.data[0].child_frame_id       = odom_msg.child_frame_id;
        tf_msg.transforms.data[0].transform.translation.x = odom_msg.pose.pose.position.x;
        tf_msg.transforms.data[0].transform.translation.y = odom_msg.pose.pose.position.y;
        tf_msg.transforms.data[0].transform.translation.z = odom_msg.pose.pose.position.z;
        tf_msg.transforms.data[0].transform.rotation      = odom_msg.pose.pose.orientation;
        tf_msg.transforms.size = 1;

        /* Local timestamp (for non-ROS diagnostics) */
        mbot_state.timestamp_us = time_us_64();
        EXIT_CRITICAL();
    }

    // Get command safely
    mbot_cmd_t local_cmd;
    mbot_state_t local_state;
    get_mbot_cmd_safe(&local_cmd);
    get_mbot_state_safe(&local_state);
    bool cmd_fresh = (time_us_64() - local_cmd.timestamp_us) < MBOT_TIMEOUT_US;
    float pwm_left = 0.0f, pwm_right = 0.0f, pwm_back = 0.0f;
    float pid_pwm_left = 0.0f, pid_pwm_right = 0.0f, pid_pwm_back = 0.0f;
    if (cmd_fresh) {
        switch (local_cmd.drive_mode) {
            case MODE_MOTOR_PWM:{
                pwm_left = local_cmd.motor_pwm[MOT_L];
                pwm_right = local_cmd.motor_pwm[MOT_R];
                pwm_back = local_cmd.motor_pwm[MOT_B];
                break;
                }
            case MODE_MOTOR_VEL:{
                // Feedforward PWM
                float vel_left_comp = local_cmd.wheel_vel[MOT_L] * params.motor_polarity[MOT_L];
                float vel_right_comp = local_cmd.wheel_vel[MOT_R] * params.motor_polarity[MOT_R];
                float vel_back_comp = local_cmd.wheel_vel[MOT_B] * params.motor_polarity[MOT_B];
                float ff_pwm_left = calibrated_pwm_from_vel_cmd(vel_left_comp, MOT_L);
                float ff_pwm_right = calibrated_pwm_from_vel_cmd(vel_right_comp, MOT_R);
                float ff_pwm_back = calibrated_pwm_from_vel_cmd(vel_back_comp, MOT_B);

                // PID PWM
                float left_correction = 0.0f, right_correction = 0.0f, back_correction = 0.0f;
                mbot_motor_vel_controller(
                    local_cmd.wheel_vel[MOT_L], local_cmd.wheel_vel[MOT_R], local_cmd.wheel_vel[MOT_B],
                    local_state.wheel_vel[MOT_L], local_state.wheel_vel[MOT_R], local_state.wheel_vel[MOT_B],
                    &left_correction, &right_correction, &back_correction
                );
                pid_pwm_left = left_correction * params.motor_polarity[MOT_L];
                pid_pwm_right = right_correction * params.motor_polarity[MOT_R];
                pid_pwm_back = back_correction * params.motor_polarity[MOT_B];
                switch (control_mode) {
                    case CONTROL_MODE_FF_ONLY:
                        pwm_left = ff_pwm_left;
                        pwm_right = ff_pwm_right;
                        pwm_back = ff_pwm_back;
                        break;
                    case CONTROL_MODE_PID_ONLY:
                        pwm_left = pid_pwm_left;
                        pwm_right = pid_pwm_right;
                        pwm_back = pid_pwm_back;
                        break;
                    case CONTROL_MODE_FF_PID:
                        pwm_left = ff_pwm_left + pid_pwm_left;
                        pwm_right = ff_pwm_right + pid_pwm_right;
                        pwm_back = ff_pwm_back + pid_pwm_back;
                        break;
                }
                break;
            }
            case MODE_MBOT_VEL: {
                // Omni inverse kinematics: body velocity -> wheel velocities
                float target_vel_left  = (SQRT3 / 2.0f * local_cmd.vx - 0.5f * local_cmd.vy - OMNI_BASE_RADIUS * local_cmd.wz) / OMNI_WHEEL_RADIUS;
                float target_vel_right = (-SQRT3 / 2.0f * local_cmd.vx - 0.5f * local_cmd.vy - OMNI_BASE_RADIUS * local_cmd.wz) / OMNI_WHEEL_RADIUS;
                float target_vel_back  = (local_cmd.vy - OMNI_BASE_RADIUS * local_cmd.wz) / OMNI_WHEEL_RADIUS;

                // Feedforward PWM
                float vel_left_comp = params.motor_polarity[MOT_L] * target_vel_left;
                float vel_right_comp = params.motor_polarity[MOT_R] * target_vel_right;
                float vel_back_comp = params.motor_polarity[MOT_B] * target_vel_back;
                float ff_pwm_left = calibrated_pwm_from_vel_cmd(vel_left_comp, MOT_L);
                float ff_pwm_right = calibrated_pwm_from_vel_cmd(vel_right_comp, MOT_R);
                float ff_pwm_back = calibrated_pwm_from_vel_cmd(vel_back_comp, MOT_B);

                // PID PWM
                float left_correction = 0.0f, right_correction = 0.0f, back_correction = 0.0f;
                mbot_motor_vel_controller(
                    target_vel_left, target_vel_right, target_vel_back,
                    local_state.wheel_vel[MOT_L], local_state.wheel_vel[MOT_R], local_state.wheel_vel[MOT_B],
                    &left_correction, &right_correction, &back_correction
                );

                // Apply motor polarity to PID corrections
                pid_pwm_left = params.motor_polarity[MOT_L] * left_correction;
                pid_pwm_right = params.motor_polarity[MOT_R] * right_correction;
                pid_pwm_back = params.motor_polarity[MOT_B] * back_correction;

                switch (control_mode) {
                    case CONTROL_MODE_FF_ONLY:
                        pwm_left = ff_pwm_left;
                        pwm_right = ff_pwm_right;
                        pwm_back = ff_pwm_back;
                        break;
                    case CONTROL_MODE_PID_ONLY:
                        pwm_left = pid_pwm_left;
                        pwm_right = pid_pwm_right;
                        pwm_back = pid_pwm_back;
                        break;
                    case CONTROL_MODE_FF_PID:
                        pwm_left = ff_pwm_left + pid_pwm_left;
                        pwm_right = ff_pwm_right + pid_pwm_right;
                        pwm_back = ff_pwm_back + pid_pwm_back;
                        break;
                }
                break;
            }
            default:
                pwm_left = 0.0f;
                pwm_right = 0.0f;
                pwm_back = 0.0f;
        }
    } else {
        pwm_left = 0.0f;
        pwm_right = 0.0f;
        pwm_back = 0.0f;
    }
    // Low-pass filter if enabled
    if (enable_pwm_lpf) {
        pwm_left = rc_filter_march(&mbot_left_pwm_lpf, pwm_left);
        pwm_right = rc_filter_march(&mbot_right_pwm_lpf, pwm_right);
        pwm_back = rc_filter_march(&mbot_back_pwm_lpf, pwm_back);
    }

    // Set motors
    mbot_motor_set_duty(MOT_L, pwm_left);
    mbot_motor_set_duty(MOT_R, pwm_right);
    mbot_motor_set_duty(MOT_B, pwm_back);

    {   // Critical section for storing motor PWM to mbot_state
        ENTER_CRITICAL();
        mbot_state.motor_pwm[MOT_L] = pwm_left;
        mbot_state.motor_pwm[MOT_R] = pwm_right;
        mbot_state.motor_pwm[MOT_B] = pwm_back;
        EXIT_CRITICAL();
    }

    return true;
}

// Timer callback for periodic ROS publishing (runs at ROS_TIMER_HZ)
void timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
    // Periodic time re-sync to prevent clock drift
    static uint64_t last_sync_time_us = 0;
    uint64_t now_us = time_us_64();
    if (now_us - last_sync_time_us >= TIME_SYNC_INTERVAL_SEC * 1000000) {
        last_sync_time_us = now_us;
        rmw_uros_sync_session(100);
    }

    mbot_publish_state();
}

/**
 * @brief Main entry point
 */
int main() {
    // Initialize Dual CDC and stdio
    stdio_init_all();
    dual_cdc_init();
    // Launch USB-servicing loop on core 1
    multicore_launch_core1(core1_usb_task);
    mbot_wait_ms(2000);

    printf("\r\nMBot Omni Firmware (ROS2)\r\n");
    printf("--------------------------------\r\n");

    mbot_init_hardware();

    mbot_read_fram(0, sizeof(params), (uint8_t*)&params);
    mbot_read_pid_gains(&params);
    mbot_controller_init();

    printf("\nCalibration Parameters:\n");
    print_mbot_params(&params);
    mbot_wait_ms(1000);

    int validate_status = validate_mbot_omni_FRAM_data(&params, MOT_L, MOT_R, MOT_B);
    if (validate_status < 0){
        printf("Failed to validate FRAM Data! Error code: %d\n", validate_status);
    }

    printf("\nStarting MBot Loop...\n");
    mbot_state.last_encoder_time = time_us_64();
    if (!add_repeating_timer_ms((int32_t)(MAIN_LOOP_PERIOD * 1000.0f), mbot_loop, NULL, &mbot_loop_timer)){
        printf("Failed to add control loop timer! Halting.\r\n");
        while(1) {tight_loop_contents();}
    }

    rmw_ret_t rmw_ret = rmw_uros_set_custom_transport(
        true,
        NULL,
        pico_serial_transport_open,
        pico_serial_transport_close,
        pico_serial_transport_write,
        pico_serial_transport_read
    );
    if (rmw_ret != RMW_RET_OK) {
        printf("rmw_uros_set_custom_transport failed: %d\n", rmw_ret);
        while(1) {tight_loop_contents();}
    }

    printf("Pinging Agent...\n");
    // Wait for agent successful ping for 3 minutes.
    const int timeout_ms = 1000;
    const uint8_t attempts = 180;
    rcl_ret_t ping_ret = rmw_uros_ping_agent(timeout_ms, attempts);
    if (ping_ret != RMW_RET_OK) {
        printf("[FATAL] Failed to ping Micro-ROS Agent!\n");
        printf("[FATAL] Please re-start the agent and press the reset button.\n");
        while(1) {tight_loop_contents();}
    }

    printf("Initializing Micro-ROS...\n");
    if (mbot_init_micro_ros() != MBOT_OK) {
        printf("[FATAL] Micro-ROS initialization failed\n");
        while(1) {tight_loop_contents();}
    }

    printf("Done Booting Up!\n");
    fflush(stdout);

    // Main loop: if a fatal error occurs, halt and wait for reset
    static int64_t last_200ms_time = 0;
    while (1) {
        if (mbot_spin_micro_ros() != MBOT_OK) {
            printf("[FATAL] Micro-ROS spin failed. Please check agent connection and press the reset button.\n");
            while(1) { tight_loop_contents(); }
        }

        if (time_us_64() - last_200ms_time > 200000) { // 200ms interval
            last_200ms_time = time_us_64();
            mbot_print_state(&mbot_state);
        }
        sleep_us(500);
    }
    return 0;
}

/******************************************************
 * Helper Functions
 ******************************************************/
static int mbot_init_hardware(void){
    printf("Initializing Hardware...\n");
    // Initialize Motors (3 for omni)
    mbot_motor_init(MOT_L);
    mbot_motor_init(MOT_R);
    mbot_motor_init(MOT_B);
    mbot_encoder_init();

    // Initialize the IMU
    mbot_imu_config = mbot_imu_default_config();
    mbot_imu_config.sample_rate = 200;
    mbot_imu_init(&mbot_imu_data, mbot_imu_config);

    // Initialize ADC
    adc_init();
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    adc_gpio_init(29);

    // Initialize PWM LPFs for smoother motion (3 for omni)
    mbot_left_pwm_lpf = rc_filter_empty();
    mbot_right_pwm_lpf = rc_filter_empty();
    mbot_back_pwm_lpf = rc_filter_empty();
    rc_filter_first_order_lowpass(&mbot_left_pwm_lpf, MAIN_LOOP_PERIOD, 0.20f);
    rc_filter_first_order_lowpass(&mbot_right_pwm_lpf, MAIN_LOOP_PERIOD, 0.20f);
    rc_filter_first_order_lowpass(&mbot_back_pwm_lpf, MAIN_LOOP_PERIOD, 0.20f);

    // Initialize FRAM
    mbot_init_fram();
    return MBOT_OK;
}

static void mbot_read_imu(void) {
    for(int i = 0; i < 3; i++) {
        mbot_state.imu_gyro[i] = mbot_imu_data.gyro[i];
        mbot_state.imu_accel[i] = mbot_imu_data.accel[i];
        mbot_state.imu_mag[i] = mbot_imu_data.mag[i];
        mbot_state.imu_rpy[i] = mbot_imu_data.rpy[i];
    }
    for(int i = 0; i < 4; i++) {
        mbot_state.imu_quat[i] = mbot_imu_data.quat[i];
    }
    /* Populate IMU ROS message with acquisition-time stamp */
    int64_t stamp_ns = rmw_uros_epoch_nanos();

    imu_msg.header.stamp.sec     = stamp_ns / 1000000000LL;
    imu_msg.header.stamp.nanosec = stamp_ns % 1000000000LL;

    imu_msg.angular_velocity.x = mbot_state.imu_gyro[0];
    imu_msg.angular_velocity.y = mbot_state.imu_gyro[1];
    imu_msg.angular_velocity.z = mbot_state.imu_gyro[2];

    imu_msg.linear_acceleration.x = mbot_state.imu_accel[0];
    imu_msg.linear_acceleration.y = mbot_state.imu_accel[1];
    imu_msg.linear_acceleration.z = mbot_state.imu_accel[2];

    imu_msg.orientation.w = mbot_state.imu_quat[0];
    imu_msg.orientation.x = mbot_state.imu_quat[1];
    imu_msg.orientation.y = mbot_state.imu_quat[2];
    imu_msg.orientation.z = mbot_state.imu_quat[3];
}

static void mbot_read_encoders(void) {
    int64_t now = time_us_64();

    // Calculate actual delta time since last encoder read
    mbot_state.encoder_delta_t = now - mbot_state.last_encoder_time;

    // If dt is zero or negative, use nominal period
    if (mbot_state.encoder_delta_t <= 0) {
        mbot_state.encoder_delta_t = ((int64_t)(MAIN_LOOP_PERIOD * 1000000.0f));
    }

    mbot_state.last_encoder_time = now;
    mbot_state.encoder_ticks[MOT_L] = mbot_encoder_read_count(MOT_L);
    mbot_state.encoder_ticks[MOT_R] = mbot_encoder_read_count(MOT_R);
    mbot_state.encoder_ticks[MOT_B] = mbot_encoder_read_count(MOT_B);
    mbot_state.encoder_delta_ticks[MOT_L] = mbot_encoder_read_delta(MOT_L);
    mbot_state.encoder_delta_ticks[MOT_R] = mbot_encoder_read_delta(MOT_R);
    mbot_state.encoder_delta_ticks[MOT_B] = mbot_encoder_read_delta(MOT_B);

    int64_t stamp_ns = rmw_uros_epoch_nanos();
    encoders_msg.stamp.sec     = stamp_ns / 1000000000LL;
    encoders_msg.stamp.nanosec = stamp_ns % 1000000000LL;

    for (int i = 0; i < 3; i++) {
        encoders_msg.ticks[i]      = mbot_state.encoder_ticks[i];
        encoders_msg.delta_ticks[i] = mbot_state.encoder_delta_ticks[i];
    }

    encoders_msg.delta_time = (int32_t)mbot_state.encoder_delta_t;
}

static void mbot_read_adc(void) {
    int64_t stamp_ns = rmw_uros_epoch_nanos();
    battery_msg.stamp.sec     = stamp_ns / 1000000000LL;
    battery_msg.stamp.nanosec = stamp_ns % 1000000000LL;
    int16_t raw;
    for(int i = 0; i < 4; i++) {
        adc_select_input(i);
        raw = adc_read();
        battery_msg.raw[i] = raw;
        mbot_state.analog_in[i] = adc_conversion_factor * raw;
        battery_msg.volts[i] = adc_conversion_factor * raw;
    }
    // last channel is battery voltage (has 5x divider)
    mbot_state.analog_in[3] = 5.0f * adc_conversion_factor * raw;
    battery_msg.volts[3] = 5.0f * adc_conversion_factor * raw;
}

static float calibrated_pwm_from_vel_cmd(float vel_cmd, int motor_idx) {
    if (vel_cmd > 0.0f) {
        return (vel_cmd * params.slope_pos[motor_idx]) + params.itrcpt_pos[motor_idx];
    } else if (vel_cmd < 0.0f) {
        return (vel_cmd * params.slope_neg[motor_idx]) + params.itrcpt_neg[motor_idx];
    }
    return 0.0f;
}

static void mbot_calculate_motor_vel(void) {
    float conversion = (1.0f / GEAR_RATIO) * (1.0f / ENCODER_RES) * 1E6f * 2.0f * PI;
    int64_t delta_t = mbot_state.encoder_delta_t;

    if (delta_t <= 0) {
        mbot_state.wheel_vel[MOT_L] = 0.0f;
        mbot_state.wheel_vel[MOT_R] = 0.0f;
        mbot_state.wheel_vel[MOT_B] = 0.0f;
        return;
    }

    mbot_state.wheel_vel[MOT_L] = params.encoder_polarity[MOT_L] *
        (conversion / delta_t) * mbot_state.encoder_delta_ticks[MOT_L];
    mbot_state.wheel_vel[MOT_R] = params.encoder_polarity[MOT_R] *
        (conversion / delta_t) * mbot_state.encoder_delta_ticks[MOT_R];
    mbot_state.wheel_vel[MOT_B] = params.encoder_polarity[MOT_B] *
        (conversion / delta_t) * mbot_state.encoder_delta_ticks[MOT_B];
}

// Omni forward kinematics: wheel velocities -> body velocity
static void mbot_calculate_omni_body_vel(float wheel_left_vel, float wheel_right_vel, float wheel_back_vel, float* vx, float* vy, float* wz) {
    *vx = OMNI_WHEEL_RADIUS * (wheel_left_vel * INV_SQRT3 - wheel_right_vel * INV_SQRT3);
    *vy = OMNI_WHEEL_RADIUS * (-wheel_left_vel / 3.0f - wheel_right_vel / 3.0f + wheel_back_vel * (2.0f / 3.0f));
    *wz = OMNI_WHEEL_RADIUS * -(wheel_left_vel + wheel_right_vel + wheel_back_vel) / (3.0f * OMNI_BASE_RADIUS);
}

static void print_mbot_params(const mbot_params_t* params) {
    printf("Motor Polarity: %d %d %d\n", params->motor_polarity[MOT_L], params->motor_polarity[MOT_R], params->motor_polarity[MOT_B]);
    printf("Encoder Polarity: %d %d %d\n", params->encoder_polarity[MOT_L], params->encoder_polarity[MOT_R], params->encoder_polarity[MOT_B]);
    printf("Positive Slope: %f %f %f\n", params->slope_pos[MOT_L], params->slope_pos[MOT_R], params->slope_pos[MOT_B]);
    printf("Positive Intercept: %f %f %f\n", params->itrcpt_pos[MOT_L], params->itrcpt_pos[MOT_R], params->itrcpt_pos[MOT_B]);
    printf("Negative Slope: %f %f %f\n", params->slope_neg[MOT_L], params->slope_neg[MOT_R], params->slope_neg[MOT_B]);
    printf("Negative Intercept: %f %f %f\n", params->itrcpt_neg[MOT_L], params->itrcpt_neg[MOT_R], params->itrcpt_neg[MOT_B]);
    printf("\nPID Gains (kp, ki, kd, tf):\n");
    printf("  Wheel L : %f %f %f %f\n", params->left_wheel_vel_pid[0], params->left_wheel_vel_pid[1], params->left_wheel_vel_pid[2], params->left_wheel_vel_pid[3]);
    printf("  Wheel R : %f %f %f %f\n", params->right_wheel_vel_pid[0], params->right_wheel_vel_pid[1], params->right_wheel_vel_pid[2], params->right_wheel_vel_pid[3]);
    printf("  Wheel B : %f %f %f %f\n", params->back_wheel_vel_pid[0], params->back_wheel_vel_pid[1], params->back_wheel_vel_pid[2], params->back_wheel_vel_pid[3]);

    const char* mode_str = "UNKNOWN";
    if(params->control_mode == 0) mode_str = "Feedforward Only";
    else if(params->control_mode == 1) mode_str = "PID Only";
    else if(params->control_mode == 2) mode_str = "FF + PID";

    printf("\nControl Mode: %d (%s)\n", params->control_mode, mode_str);
}

static void get_mbot_state_safe(mbot_state_t* dest) {
    ENTER_CRITICAL();
    *dest = mbot_state;
    EXIT_CRITICAL();
}
static void set_mbot_state_safe(const mbot_state_t* src) {
    ENTER_CRITICAL();
    mbot_state = *src;
    EXIT_CRITICAL();
}
static void get_mbot_cmd_safe(mbot_cmd_t* dest) {
    ENTER_CRITICAL();
    *dest = mbot_cmd;
    EXIT_CRITICAL();
}
static void set_mbot_cmd_safe(const mbot_cmd_t* src) {
    ENTER_CRITICAL();
    mbot_cmd = *src;
    EXIT_CRITICAL();
}

static void core1_usb_task(void) {
    /* Runs on core 1: keep TinyUSB CDC endpoints serviced */
    while (true) {
        dual_cdc_task();
        sleep_us(100); /* slight yield to other IRQs */
    }
}
