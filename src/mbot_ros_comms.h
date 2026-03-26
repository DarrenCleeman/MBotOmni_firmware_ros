#ifndef MBOT_ROS_COMMS_H
#define MBOT_ROS_COMMS_H

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/transform_stamped.h>
#include <tf2_msgs/msg/tf_message.h>
#include <mbot_interfaces/msg/encoders.h>
#include <mbot_interfaces/msg/motor_velocity.h>
#include <mbot_interfaces/msg/motor_pwm.h>
#include <mbot_interfaces/msg/battery_adc.h>
#include <std_srvs/srv/trigger.h>

// Extern declarations for ROS objects

// Publishers
extern rcl_publisher_t imu_publisher;
extern rcl_publisher_t odom_publisher;
extern rcl_publisher_t motor_vel_publisher;
extern rcl_publisher_t tf_publisher;
extern rcl_publisher_t encoders_publisher;
extern rcl_publisher_t battery_publisher;

// Published Messages
extern sensor_msgs__msg__Imu imu_msg;
extern nav_msgs__msg__Odometry odom_msg;
extern mbot_interfaces__msg__MotorVelocity motor_vel_msg;
extern tf2_msgs__msg__TFMessage tf_msg;
extern mbot_interfaces__msg__Encoders encoders_msg;
extern mbot_interfaces__msg__BatteryADC battery_msg;

// Subscribers
extern rcl_subscription_t cmd_vel_subscriber;
extern rcl_subscription_t motor_vel_cmd_subscriber;
extern rcl_subscription_t motor_pwm_cmd_subscriber;

// Subscription Message Buffers
extern geometry_msgs__msg__Twist cmd_vel_msg_buffer;
extern mbot_interfaces__msg__MotorVelocity motor_vel_cmd_msg_buffer;
extern mbot_interfaces__msg__MotorPWM motor_pwm_cmd_msg_buffer;

// Services
extern rcl_service_t reset_odometry_service;
extern std_srvs__srv__Trigger_Request reset_odom_req;
extern std_srvs__srv__Trigger_Response reset_odom_res;

// Initialization functions
int mbot_ros_comms_init_services(rcl_node_t *node);
int mbot_ros_comms_init_messages(rcl_allocator_t* allocator);
int mbot_ros_comms_init_publishers(rcl_node_t *node);
int mbot_ros_comms_init_subscribers(rcl_node_t *node);

// Callback function prototypes
void cmd_vel_callback(const void * msgin);
void motor_vel_cmd_callback(const void * msgin);
void motor_pwm_cmd_callback(const void * msgin);
void reset_odometry_callback(const void * req, void * res);

// Helper to add comms to executor
int mbot_ros_comms_add_to_executor(rclc_executor_t *executor);

#endif // MBOT_ROS_COMMS_H
