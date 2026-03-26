#include "mbot_controller.h"
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <mbot/defs/mbot_params.h>
#include "config/mbot_classic_config.h"
#include "config/mbot_classic_default_pid.h"
#include <rc/math/filter.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <mbot/fram/fram.h>

// Access to global params struct defined in mbot_classic_ros.c
extern mbot_params_t params;

// Global PID controller variables
mbot_pid_config_t pid_gains = MBOT_DEFAULT_PID_GAINS;
control_mode_t control_mode = (control_mode_t)MBOT_DEFAULT_CONTROL_MODE;

static bool pid_updated = false;

void mbot_read_pid_gains(const mbot_params_t* params) {
    pid_gains.left_wheel.kp  = params->left_wheel_vel_pid[0];
    pid_gains.left_wheel.ki  = params->left_wheel_vel_pid[1];
    pid_gains.left_wheel.kd  = params->left_wheel_vel_pid[2];
    pid_gains.left_wheel.tf  = params->left_wheel_vel_pid[3];

    pid_gains.right_wheel.kp = params->right_wheel_vel_pid[0];
    pid_gains.right_wheel.ki = params->right_wheel_vel_pid[1];
    pid_gains.right_wheel.kd = params->right_wheel_vel_pid[2];
    pid_gains.right_wheel.tf = params->right_wheel_vel_pid[3];

    pid_gains.back_wheel.kp  = params->back_wheel_vel_pid[0];
    pid_gains.back_wheel.ki  = params->back_wheel_vel_pid[1];
    pid_gains.back_wheel.kd  = params->back_wheel_vel_pid[2];
    pid_gains.back_wheel.tf  = params->back_wheel_vel_pid[3];

    control_mode = (control_mode_t)params->control_mode;
}

// PID filters
rc_filter_t left_wheel_pid;
rc_filter_t right_wheel_pid;
rc_filter_t back_wheel_pid;
rc_filter_t body_vel_vx_pid;
rc_filter_t body_vel_wz_pid;

int mbot_controller_init(void) {
    left_wheel_pid = rc_filter_empty();
    right_wheel_pid = rc_filter_empty();
    back_wheel_pid = rc_filter_empty();
    body_vel_vx_pid = rc_filter_empty();
    body_vel_wz_pid = rc_filter_empty();

    rc_filter_pid(&left_wheel_pid,
                  pid_gains.left_wheel.kp,
                  pid_gains.left_wheel.ki,
                  pid_gains.left_wheel.kd,
                  pid_gains.left_wheel.tf,
                  MAIN_LOOP_PERIOD);

    rc_filter_pid(&right_wheel_pid,
                  pid_gains.right_wheel.kp,
                  pid_gains.right_wheel.ki,
                  pid_gains.right_wheel.kd,
                  pid_gains.right_wheel.tf,
                  MAIN_LOOP_PERIOD);

    rc_filter_pid(&back_wheel_pid,
                  pid_gains.back_wheel.kp,
                  pid_gains.back_wheel.ki,
                  pid_gains.back_wheel.kd,
                  pid_gains.back_wheel.tf,
                  MAIN_LOOP_PERIOD);

    rc_filter_enable_saturation(&left_wheel_pid, -1.0, 1.0);
    rc_filter_enable_saturation(&right_wheel_pid, -1.0, 1.0);
    rc_filter_enable_saturation(&back_wheel_pid, -1.0, 1.0);

    return MBOT_OK;
}

void mbot_motor_vel_controller(float target_left_vel, float target_right_vel, float target_back_vel,
                              float current_left_vel, float current_right_vel, float current_back_vel,
                              float* left_correction, float* right_correction, float* back_correction) {
    float left_error = target_left_vel - current_left_vel;
    float right_error = target_right_vel - current_right_vel;
    float back_error = target_back_vel - current_back_vel;

    *left_correction = rc_filter_march(&left_wheel_pid, left_error);
    *right_correction = rc_filter_march(&right_wheel_pid, right_error);
    *back_correction = rc_filter_march(&back_wheel_pid, back_error);
}

int init_parameter_server(rclc_parameter_server_t* parameter_server, rcl_node_t* node) {
    rcl_ret_t ret;

    rclc_parameter_options_t options = {
        .notify_changed_over_dds = false,
        .max_params = 13,
        .allow_undeclared_parameters = false,
        .low_mem_mode = false
    };

    ret = rclc_parameter_server_init_with_option(parameter_server, node, &options);
    if (ret != RCL_RET_OK) {
        printf("[FATAL] Failed to init parameter server: %d\n", ret);
        return MBOT_ERROR;
    }

    // Add parameters for all PID gains (left, right, back wheels)
    ret = rclc_add_parameter(parameter_server, "left_wheel.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "left_wheel.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "left_wheel.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "left_wheel.tf", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_add_parameter(parameter_server, "right_wheel.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "right_wheel.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "right_wheel.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "right_wheel.tf", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_add_parameter(parameter_server, "back_wheel.kp", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "back_wheel.ki", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "back_wheel.kd", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_add_parameter(parameter_server, "back_wheel.tf", RCLC_PARAMETER_DOUBLE);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_add_parameter(parameter_server, "control_mode", RCLC_PARAMETER_INT);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    // Set initial values
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.kp", pid_gains.left_wheel.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.ki", pid_gains.left_wheel.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.kd", pid_gains.left_wheel.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "left_wheel.tf", pid_gains.left_wheel.tf);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_parameter_set_double(parameter_server, "right_wheel.kp", pid_gains.right_wheel.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "right_wheel.ki", pid_gains.right_wheel.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "right_wheel.kd", pid_gains.right_wheel.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "right_wheel.tf", pid_gains.right_wheel.tf);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_parameter_set_double(parameter_server, "back_wheel.kp", pid_gains.back_wheel.kp);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "back_wheel.ki", pid_gains.back_wheel.ki);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "back_wheel.kd", pid_gains.back_wheel.kd);
    if (ret != RCL_RET_OK) return MBOT_ERROR;
    ret = rclc_parameter_set_double(parameter_server, "back_wheel.tf", pid_gains.back_wheel.tf);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    ret = rclc_parameter_set_int(parameter_server, "control_mode", control_mode);
    if (ret != RCL_RET_OK) return MBOT_ERROR;

    return MBOT_OK;
}

bool parameter_callback(const Parameter * old_param, const Parameter * new_param, void * context) {
    if (new_param == NULL) {
        return false;
    }

    const char* param_name = new_param->name.data;

    if (strcmp(param_name, "left_wheel.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.kp = new_param->value.double_value;
            rc_filter_pid(&left_wheel_pid, pid_gains.left_wheel.kp, pid_gains.left_wheel.ki,
                          pid_gains.left_wheel.kd, pid_gains.left_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&left_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "left_wheel.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.ki = new_param->value.double_value;
            rc_filter_pid(&left_wheel_pid, pid_gains.left_wheel.kp, pid_gains.left_wheel.ki,
                          pid_gains.left_wheel.kd, pid_gains.left_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&left_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "left_wheel.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.kd = new_param->value.double_value;
            rc_filter_pid(&left_wheel_pid, pid_gains.left_wheel.kp, pid_gains.left_wheel.ki,
                          pid_gains.left_wheel.kd, pid_gains.left_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&left_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "left_wheel.tf") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.left_wheel.tf = new_param->value.double_value;
            rc_filter_pid(&left_wheel_pid, pid_gains.left_wheel.kp, pid_gains.left_wheel.ki,
                          pid_gains.left_wheel.kd, pid_gains.left_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&left_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.kp = new_param->value.double_value;
            rc_filter_pid(&right_wheel_pid, pid_gains.right_wheel.kp, pid_gains.right_wheel.ki,
                          pid_gains.right_wheel.kd, pid_gains.right_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&right_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.ki = new_param->value.double_value;
            rc_filter_pid(&right_wheel_pid, pid_gains.right_wheel.kp, pid_gains.right_wheel.ki,
                          pid_gains.right_wheel.kd, pid_gains.right_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&right_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.kd = new_param->value.double_value;
            rc_filter_pid(&right_wheel_pid, pid_gains.right_wheel.kp, pid_gains.right_wheel.ki,
                          pid_gains.right_wheel.kd, pid_gains.right_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&right_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "right_wheel.tf") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.right_wheel.tf = new_param->value.double_value;
            rc_filter_pid(&right_wheel_pid, pid_gains.right_wheel.kp, pid_gains.right_wheel.ki,
                          pid_gains.right_wheel.kd, pid_gains.right_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&right_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "back_wheel.kp") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.back_wheel.kp = new_param->value.double_value;
            rc_filter_pid(&back_wheel_pid, pid_gains.back_wheel.kp, pid_gains.back_wheel.ki,
                          pid_gains.back_wheel.kd, pid_gains.back_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&back_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "back_wheel.ki") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.back_wheel.ki = new_param->value.double_value;
            rc_filter_pid(&back_wheel_pid, pid_gains.back_wheel.kp, pid_gains.back_wheel.ki,
                          pid_gains.back_wheel.kd, pid_gains.back_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&back_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "back_wheel.kd") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.back_wheel.kd = new_param->value.double_value;
            rc_filter_pid(&back_wheel_pid, pid_gains.back_wheel.kp, pid_gains.back_wheel.ki,
                          pid_gains.back_wheel.kd, pid_gains.back_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&back_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "back_wheel.tf") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_DOUBLE) {
            pid_gains.back_wheel.tf = new_param->value.double_value;
            rc_filter_pid(&back_wheel_pid, pid_gains.back_wheel.kp, pid_gains.back_wheel.ki,
                          pid_gains.back_wheel.kd, pid_gains.back_wheel.tf, MAIN_LOOP_PERIOD);
            rc_filter_enable_saturation(&back_wheel_pid, -1.0, 1.0);
            pid_updated = true;
        }
    } else if (strcmp(param_name, "control_mode") == 0) {
        if (new_param->value.type == RCLC_PARAMETER_INT) {
            int val = new_param->value.integer_value;
            control_mode = (control_mode_t)val;
            pid_updated = true;
        }
    }

    if (pid_updated) {
        mbot_save_params_to_fram();
    }
    return pid_updated;
}

// Save current PID gains and control mode to FRAM
int mbot_save_params_to_fram(void) {
    params.left_wheel_vel_pid[0]  = pid_gains.left_wheel.kp;
    params.left_wheel_vel_pid[1]  = pid_gains.left_wheel.ki;
    params.left_wheel_vel_pid[2]  = pid_gains.left_wheel.kd;
    params.left_wheel_vel_pid[3]  = pid_gains.left_wheel.tf;

    params.right_wheel_vel_pid[0] = pid_gains.right_wheel.kp;
    params.right_wheel_vel_pid[1] = pid_gains.right_wheel.ki;
    params.right_wheel_vel_pid[2] = pid_gains.right_wheel.kd;
    params.right_wheel_vel_pid[3] = pid_gains.right_wheel.tf;

    params.back_wheel_vel_pid[0]  = pid_gains.back_wheel.kp;
    params.back_wheel_vel_pid[1]  = pid_gains.back_wheel.ki;
    params.back_wheel_vel_pid[2]  = pid_gains.back_wheel.kd;
    params.back_wheel_vel_pid[3]  = pid_gains.back_wheel.tf;

    params.control_mode = control_mode;

    // Write entire struct to FRAM
    return mbot_write_fram(0, sizeof(params), (uint8_t*)&params);
}
