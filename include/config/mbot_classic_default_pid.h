#ifndef CONFIG_MBOT_OMNI_DEFAULT_PID_H
#define CONFIG_MBOT_OMNI_DEFAULT_PID_H

#include "../../src/mbot_controller.h"

// Default PID gains for omni bot (3 wheels)
static const mbot_pid_config_t MBOT_DEFAULT_PID_GAINS = {
    .left_wheel  = { .kp = 0.1f, .ki = 0.0f, .kd = 0.01f, .tf = 0.1f },
    .right_wheel = { .kp = 0.1f, .ki = 0.0f, .kd = 0.01f, .tf = 0.1f },
    .back_wheel  = { .kp = 0.1f, .ki = 0.0f, .kd = 0.01f, .tf = 0.1f },
};

static const int MBOT_DEFAULT_CONTROL_MODE = 2; // 0=FF only, 1=PID only, 2=FF+PID

#endif /* CONFIG_MBOT_OMNI_DEFAULT_PID_H */