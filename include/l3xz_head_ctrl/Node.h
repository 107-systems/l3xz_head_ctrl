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

#include <memory>

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <l3xz_head_ctrl/head/HeadController.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Node : public rclcpp::Node
{
public:
  Node();

private:
  std::unique_ptr<head::Controller> _head_ctrl;
  dynamixelplusplus::Dynamixel::Id _pan_servo_id, _tilt_servo_id;
  float _pan_angular_velocity, _tilt_angular_velocity;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _head_sub;
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;

  void onCtrlLoopTimerEvent();

  void updateHeadControllerInput(geometry_msgs::msg::Twist::SharedPtr const msg);

  static int                              constexpr DEFAULT_SERIAL_BAUDRATE          = 115200;
  static dynamixelplusplus::Dynamixel::Id constexpr DEFAULT_PAN_SERVO_ID             = 7;
  static dynamixelplusplus::Dynamixel::Id constexpr DEFAULT_TILT_SERVO_ID            = 8;
  static float                            constexpr DEFAULT_PAN_SERVO_INITIAL_ANGLE  = 180.0f;
  static float                            constexpr DEFAULT_TILT_SERVO_INITIAL_ANGLE = 180.0f;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* L3XZ_HEAD_CTRL_NODE_H_ */
