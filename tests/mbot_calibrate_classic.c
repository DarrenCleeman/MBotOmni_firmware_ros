#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <pico/stdlib.h>
#include <pico/binary_info.h>
#include <mbot/imu/imu.h>
#include <mbot/motor/motor.h>
#include <mbot/encoder/encoder.h>
#include <mbot/fram/fram.h>
#include <mbot/defs/mbot_params.h>

// for USB CDC support
#include <pico/multicore.h>
#include <comms/dual_cdc.h>

#include "config/mbot_classic_config.h"
#include "config/mbot_classic_default_pid.h"

mbot_bhy_data_t mbot_imu_data;
mbot_bhy_config_t mbot_imu_config;

int find_index_of_max_positive(float* arr, int size) {
    float max_positive = -5000.0;
    int idx = -1;
    for (int i = 0; i < size; i++) {
        if (arr[i] > max_positive && arr[i] > 0) {
            max_positive = arr[i];
            idx = i;
        }
    }
    return idx;
}

void least_squares_fit(float* pwms, float* speeds, int n, float* m, float* b) {
    float sum_x = 0.0, sum_y = 0.0, sum_xy = 0.0, sum_xx = 0.0;

    for (int i = 0; i < n; ++i) {
        if (speeds[i] != 0.0) {
            sum_x += speeds[i];
            sum_y += pwms[i];
            sum_xy += speeds[i] * pwms[i];
            sum_xx += speeds[i] * speeds[i];
        }
    }

    *m = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x * sum_x);
    *b = (sum_y - *m * sum_x) / n;
}

void print_mbot_params_omni(const mbot_params_t* params) {
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

static void core1_usb_task(void) {
    while (true) {
        dual_cdc_task();
        sleep_us(100);
    }
}

int main() {
    // Initialization
    mbot_params_t params;
    stdio_init_all();
    dual_cdc_init();
    multicore_launch_core1(core1_usb_task);
    printf("\n\n\nInitializing...\n");
    bi_decl(bi_program_description("Calibration for MBot Omni (3-wheel omnidirectional)"));
    mbot_motor_init(MOT_L);
    mbot_motor_init(MOT_R);
    mbot_motor_init(MOT_B);
    mbot_encoder_init();
    mbot_init_fram();
    printf("\nWaiting for 5 seconds...\n");
    sleep_ms(5000);

    // ====================================================
    // Find Encoder Polarity
    // ====================================================
    printf("\nTesting Encoder Polarity...\n");
    mbot_motor_set_duty(MOT_L, 0.2);
    mbot_motor_set_duty(MOT_R, 0.2);
    mbot_motor_set_duty(MOT_B, 0.2);
    for(int i=0; i<5; i++){
        printf("E0: %d , E1: %d , E2: %d\n",
            mbot_encoder_read_delta(MOT_L),
            mbot_encoder_read_delta(MOT_R),
            mbot_encoder_read_delta(MOT_B));
        sleep_ms(100);
    }
    mbot_motor_set_duty(MOT_L, 0.0);
    mbot_motor_set_duty(MOT_R, 0.0);
    mbot_motor_set_duty(MOT_B, 0.0);
    params.encoder_polarity[MOT_L] = (mbot_encoder_read_count(MOT_L)>0) ? 1 : -1;
    params.encoder_polarity[MOT_R] = (mbot_encoder_read_count(MOT_R)>0) ? 1 : -1;
    params.encoder_polarity[MOT_B] = (mbot_encoder_read_count(MOT_B)>0) ? 1 : -1;
    printf("\nENC0 POL: %d , ENC1 POL: %d , ENC2 POL: %d\n \n",
        params.encoder_polarity[MOT_L],
        params.encoder_polarity[MOT_R],
        params.encoder_polarity[MOT_B]);

    // ====================================================
    // Find Motor Polarity using IMU gyroscope
    // ====================================================
    mbot_imu_config = mbot_imu_default_config();
    mbot_imu_init(&mbot_imu_data, mbot_imu_config);
    printf("\nTesting Motor Polarity...\n");

    // For omni bot, spinning all 3 motors in the same direction causes rotation.
    // We test 8 sign combinations for 3 motors and find which gives max positive wz.
    float gyro_z[8] = {0};
    float spd = 0.4;
    float motor_duties[8][3] = {
        { spd,  spd,  spd},
        {-spd,  spd,  spd},
        { spd, -spd,  spd},
        { spd,  spd, -spd},
        {-spd, -spd,  spd},
        {-spd,  spd, -spd},
        { spd, -spd, -spd},
        {-spd, -spd, -spd}
    };

    for (int i = 0; i < 8; i++) {
        mbot_motor_set_duty(MOT_L, motor_duties[i][0]);
        mbot_motor_set_duty(MOT_R, motor_duties[i][1]);
        mbot_motor_set_duty(MOT_B, motor_duties[i][2]);

        for (int j = 0; j < 25; j++) {
            gyro_z[i] += mbot_imu_data.gyro[2];
            sleep_ms(20);
        }

        mbot_motor_set_duty(MOT_L, 0.0);
        mbot_motor_set_duty(MOT_R, 0.0);
        mbot_motor_set_duty(MOT_B, 0.0);
        printf("Combo %d - Gyro: %f\n", i, gyro_z[i]);
        sleep_ms(500);
    }

    int rot_idx = find_index_of_max_positive(gyro_z, 8);
    printf("Best rotation index = %d\n", rot_idx);

    // The combination that gives max positive wz (CCW rotation) tells us
    // that those motor signs cause CCW. For omni kinematics, all motors
    // spinning in the negative direction causes CCW, so the polarity
    // is the negation of the duty signs that caused CCW.
    params.motor_polarity[MOT_L] = (motor_duties[rot_idx][0] > 0) ? -1 : 1;
    params.motor_polarity[MOT_R] = (motor_duties[rot_idx][1] > 0) ? -1 : 1;
    params.motor_polarity[MOT_B] = (motor_duties[rot_idx][2] > 0) ? -1 : 1;

    // Adjust Encoder Polarity by motor polarity
    params.encoder_polarity[MOT_L] *= params.motor_polarity[MOT_L];
    params.encoder_polarity[MOT_R] *= params.motor_polarity[MOT_R];
    params.encoder_polarity[MOT_B] *= params.motor_polarity[MOT_B];

    printf("Motor Polarity: (%d, %d, %d)\n",
        params.motor_polarity[MOT_L],
        params.motor_polarity[MOT_R],
        params.motor_polarity[MOT_B]);

    // ====================================================
    // Calibrate slope/intercept for each motor
    // ====================================================
    printf("\nCalculating Slope and Intercept...\n");

    int num_points = 20;
    float dt = 0.5;
    float conv = (2 * M_PI)/(GEAR_RATIO * ENCODER_RES);

    // Calibrate each motor individually
    int motors[3] = {MOT_L, MOT_R, MOT_B};
    const char* motor_names[3] = {"Left", "Right", "Back"};

    for (int m = 0; m < 3; m++) {
        int mot = motors[m];
        float wheel_speed[num_points+1];
        float duty[num_points+1];

        // Negative direction (CCW)
        printf("\nMeasuring %s Motor - Negative Direction...\n", motor_names[m]);
        mbot_encoder_read_delta(mot);
        for(int i = 0; i <= num_points; i++){
            float d = i * 1.0/(float)num_points;
            mbot_motor_set_duty(mot, params.motor_polarity[mot] * -d);
            sleep_ms(dt * 1000);
            duty[i] = -d;
            wheel_speed[i] = conv * params.encoder_polarity[mot] * mbot_encoder_read_delta(mot) / dt;
            printf("duty: %f, speed: %f\n", duty[i], wheel_speed[i]);
        }

        // Slow down
        mbot_motor_set_duty(mot, params.motor_polarity[mot] * -0.8);
        sleep_ms(300);
        mbot_motor_set_duty(mot, params.motor_polarity[mot] * -0.5);
        sleep_ms(300);
        mbot_motor_set_duty(mot, 0.0);

        int n = num_points + 1;
        least_squares_fit(duty, wheel_speed, n, &params.slope_neg[mot], &params.itrcpt_neg[mot]);

        // Positive direction (CW)
        sleep_ms(500);
        printf("Measuring %s Motor - Positive Direction...\n", motor_names[m]);
        mbot_encoder_read_delta(mot);
        for(int i = 0; i <= num_points; i++){
            float d = i * 1.0/(float)num_points;
            mbot_motor_set_duty(mot, params.motor_polarity[mot] * d);
            sleep_ms(dt * 1000);
            duty[i] = d;
            wheel_speed[i] = conv * params.encoder_polarity[mot] * mbot_encoder_read_delta(mot) / dt;
            printf("duty: %f, speed: %f\n", duty[i], wheel_speed[i]);
        }

        // Slow down
        mbot_motor_set_duty(mot, params.motor_polarity[mot] * 0.8);
        sleep_ms(300);
        mbot_motor_set_duty(mot, params.motor_polarity[mot] * 0.5);
        sleep_ms(300);
        mbot_motor_set_duty(mot, 0.0);

        least_squares_fit(duty, wheel_speed, n, &params.slope_pos[mot], &params.itrcpt_pos[mot]);

        printf("%s Motor Calibration:\n", motor_names[m]);
        printf("  slope_pos: %f, itrcpt_pos: %f\n", params.slope_pos[mot], params.itrcpt_pos[mot]);
        printf("  slope_neg: %f, itrcpt_neg: %f\n", params.slope_neg[mot], params.itrcpt_neg[mot]);
        sleep_ms(500);
    }

    // ====================================================
    // Store default PID gains and write to FRAM
    // ====================================================
    params.left_wheel_vel_pid[0] = MBOT_DEFAULT_PID_GAINS.left_wheel.kp;
    params.left_wheel_vel_pid[1] = MBOT_DEFAULT_PID_GAINS.left_wheel.ki;
    params.left_wheel_vel_pid[2] = MBOT_DEFAULT_PID_GAINS.left_wheel.kd;
    params.left_wheel_vel_pid[3] = MBOT_DEFAULT_PID_GAINS.left_wheel.tf;

    params.right_wheel_vel_pid[0] = MBOT_DEFAULT_PID_GAINS.right_wheel.kp;
    params.right_wheel_vel_pid[1] = MBOT_DEFAULT_PID_GAINS.right_wheel.ki;
    params.right_wheel_vel_pid[2] = MBOT_DEFAULT_PID_GAINS.right_wheel.kd;
    params.right_wheel_vel_pid[3] = MBOT_DEFAULT_PID_GAINS.right_wheel.tf;

    params.back_wheel_vel_pid[0] = MBOT_DEFAULT_PID_GAINS.back_wheel.kp;
    params.back_wheel_vel_pid[1] = MBOT_DEFAULT_PID_GAINS.back_wheel.ki;
    params.back_wheel_vel_pid[2] = MBOT_DEFAULT_PID_GAINS.back_wheel.kd;
    params.back_wheel_vel_pid[3] = MBOT_DEFAULT_PID_GAINS.back_wheel.tf;

    params.control_mode = MBOT_DEFAULT_CONTROL_MODE;

    mbot_write_fram(0, sizeof(params), &params);
    mbot_params_t written;
    mbot_read_fram(0, sizeof(written), &written);
    printf("\nParameters stored in FRAM (%d bytes): \n", sizeof(written));
    print_mbot_params_omni(&written);

    printf("\nDone!\n");
    fflush(stdout);
    sleep_ms(1000);
}
