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
#include <geometry_msgs/msg/twist.hpp>

#include <ros2_dynamixel_bridge/msg/mode.hpp>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

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
    Init, Hold, Teleop
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

  class TeleopTarget
  {
  private:
    std::map<Servo, float> _angular_velocity_rad_per_sec_map;
  public:
    TeleopTarget()
    : _angular_velocity_rad_per_sec_map
    {
      {Servo::Pan,  0.0f},
      {Servo::Tilt, 0.0f},
    }
    { }
    [[nodiscard]] float angular_velocity_rps(Servo const servo) const { return _angular_velocity_rad_per_sec_map.at(servo); }
    void set_angular_velocity_rps(Servo const servo, float const ang_vel_dps) { _angular_velocity_rad_per_sec_map[servo] = ang_vel_dps; }
    [[nodiscard]] bool is_active_manual_control() const
    {
      static float constexpr ACTIVITY_EPSILON_rad_per_sec = 1.0f * M_PI / 180.0f;
      return (fabs(_angular_velocity_rad_per_sec_map.at(Servo::Pan))  > ACTIVITY_EPSILON_rad_per_sec ||
              fabs(_angular_velocity_rad_per_sec_map.at(Servo::Tilt)) > ACTIVITY_EPSILON_rad_per_sec);
    }
  };

  class ServoActual
  {
  private:
    std::map<Servo, float> _angle_rad_map;
  public:
    ServoActual()
    : _angle_rad_map
    {
      {Servo::Pan,  0.0f},
      {Servo::Tilt, 0.0f},
    } { }
    [[nodiscard]] float angle_rad(Servo const servo) const { return _angle_rad_map.at(servo); }
    void set_angle_rad(Servo const servo, float const angle_rad) { _angle_rad_map[servo] = angle_rad; }
  };

  TeleopTarget _teleop_target;
  ServoActual _servo_actual;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _head_sub;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr _pan_angle_actual_sub, _tilt_angle_actual_sub;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr _pan_angle_pub, _tilt_angle_pub, _pan_angle_vel_pub, _tilt_angle_vel_pub;
  rclcpp::Publisher<ros2_dynamixel_bridge::msg::Mode>::SharedPtr _pan_angle_mode_pub, _tilt_angle_mode_pub;

  std::chrono::steady_clock::time_point _prev_ctrl_loop_timepoint;
  static std::chrono::milliseconds constexpr CTRL_LOOP_RATE{10};
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;
  void ctrl_loop();

  std::chrono::steady_clock::time_point _prev_teleop_activity_timepoint;

  std::tuple<State, Mode, float, float, float, float> handle_Init();
  std::tuple<State, Mode, float, float, float, float> handle_Hold();
  std::tuple<State, Mode, float, float, float, float> handle_Teleop();

  void publish(Mode const mode, float const pan_rps, float const tilt_rps, float const deg, float const tilt_deg);

  static void publish_AngularVelocity     (rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr const pub, float const angular_velocity_rad_per_sec);
  static void publish_Angle               (rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr const pub, float const angle_rad);
  static void publish_mode_PositionControl(rclcpp::Publisher<ros2_dynamixel_bridge::msg::Mode>::SharedPtr const pub);
  static void publish_mode_VelocityControl(rclcpp::Publisher<ros2_dynamixel_bridge::msg::Mode>::SharedPtr const pub);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */

#endif /* L3XZ_HEAD_CTRL_NODE_H_ */
