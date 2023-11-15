/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

#ifndef L3XZ_HEAD_CTRL_NODE_H_
#define L3XZ_HEAD_CTRL_NODE_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <tuple>
#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/u_int64.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <ros2_heartbeat/publisher/Publisher.h>
#include <ros2_loop_rate_monitor/Monitor.h>

#include <ros2_dynamixel_bridge/msg/mode.hpp>

#include <mp-units/systems/si/si.h>
#include <mp-units/systems/angular/angular.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

using namespace mp_units;
using mp_units::angular::unit_symbols::deg;
using mp_units::angular::unit_symbols::rad;
using mp_units::si::unit_symbols::s;

namespace l3xz::head
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
  Node();

private:
  enum class State
  {
    Init, Startup, Hold, Teleop
  };
  State _state;

  enum class Mode
  {
    PositionControl, VelocityControl
  };

  enum class Servo
  {
    Pan, Tilt
  };

  heartbeat::Publisher::SharedPtr _heartbeat_pub;
  void init_heartbeat();

  [[nodiscard]] bool is_active_manual_control() const
  {
    auto constexpr ACTIVITY_EPSILON = 1. * deg/s;
    return (_target_angular_velocity.at(Servo::Pan)  >       ACTIVITY_EPSILON.in(rad/s) ||
            _target_angular_velocity.at(Servo::Pan)  < -1. * ACTIVITY_EPSILON.in(rad/s) ||
            _target_angular_velocity.at(Servo::Tilt) >       ACTIVITY_EPSILON.in(rad/s) ||
            _target_angular_velocity.at(Servo::Tilt) < -1. * ACTIVITY_EPSILON.in(rad/s));
  }

  rclcpp::QoS _head_qos_profile;
  rclcpp::SubscriptionOptions _head_sub_options;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _head_sub;
  std::map<Servo, quantity<rad/s>> _target_angular_velocity;
  void init_head_sub();

  std::map<Servo, quantity<rad>> _actual_angle;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _pan_angle_actual_sub, _tilt_angle_actual_sub;
  void init_sub();

  std::map<Servo, quantity<rad>> _target_angle;
  Mode _target_mode;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pan_angle_pub, _tilt_angle_pub, _pan_angle_vel_pub, _tilt_angle_vel_pub;
  rclcpp::Publisher<ros2_dynamixel_bridge::msg::Mode>::SharedPtr _pan_angle_mode_pub, _tilt_angle_mode_pub;
  void init_pub();

  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  loop_rate::Monitor::SharedPtr _ctrl_loop_rate_monitor;
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void ctrl_loop();

  std::chrono::steady_clock::time_point _prev_teleop_activity_timepoint;

  State handle_Init();
  State handle_Startup();
  State handle_Hold();
  State handle_Teleop();

  void publish();

  static void publish_AngularVelocity     (rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr const pub, quantity<rad/s> const angular_velocity);
  static void publish_Angle               (rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr const pub, quantity<rad> const angle);
  static void publish_mode_PositionControl(rclcpp::Publisher<ros2_dynamixel_bridge::msg::Mode>::SharedPtr const pub);
  static void publish_mode_VelocityControl(rclcpp::Publisher<ros2_dynamixel_bridge::msg::Mode>::SharedPtr const pub);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */

#endif /* L3XZ_HEAD_CTRL_NODE_H_ */
