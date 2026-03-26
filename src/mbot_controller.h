#ifndef MBOT_CONTROLLER_H
#define MBOT_CONTROLLER_H

#include <stdbool.h>
#include <rclc_parameter/rclc_parameter.h>
#include <mbot/defs/mbot_params.h>

typedef struct {
    float kp;
    float ki;
    float kd;
    float tf;
} pid_params_t;

typedef struct {
    pid_params_t left_wheel;
    pid_params_t right_wheel;
    pid_params_t back_wheel;
} mbot_pid_config_t;

typedef enum {
    CONTROL_MODE_FF_ONLY,
    CONTROL_MODE_PID_ONLY,
    CONTROL_MODE_FF_PID
} control_mode_t;

// Parameter server
int init_parameter_server(rclc_parameter_server_t* parameter_server, rcl_node_t* node);
bool parameter_callback(const Parameter * old_param, const Parameter * new_param, void * context);

int mbot_controller_init(void);
void mbot_motor_vel_controller(float target_left_vel, float target_right_vel, float target_back_vel,
                              float current_left_vel, float current_right_vel, float current_back_vel,
                              float* left_correction, float* right_correction, float* back_correction);

// Populate global pid_gains from values stored in mbot_params_t
void mbot_read_pid_gains(const mbot_params_t* params);

// PID gains global config
extern mbot_pid_config_t pid_gains;
extern control_mode_t control_mode;

// Save current PID gains and control mode to FRAM
int mbot_save_params_to_fram(void);
#endif