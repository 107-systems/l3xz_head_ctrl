/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

#ifndef L3XZ_HEAD_CTRL_NODE_H_
#define L3XZ_HEAD_CTRL_NODE_H_

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/twist.hpp>

#include <l3xz_head_ctrl/msg/head_angle.hpp>

#include <l3xz_head_ctrl/head/HeadController.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class HeadControlNode : public rclcpp::Node
{
public:
  HeadControlNode();

private:
  head::Controller _head_ctrl;
  head::ControllerInput _head_ctrl_input;
  head::ControllerOutput _head_ctrl_output;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _head_sub;
  rclcpp::Publisher<l3xz_head_ctrl::msg::HeadAngle>::SharedPtr _head_angle_pub;
  rclcpp::Subscription<l3xz_head_ctrl::msg::HeadAngle>::SharedPtr _head_angle_sub;
  rclcpp::TimerBase::SharedPtr _ctrl_loop_timer;

  void onCtrlLoopTimerEvent();

  void updateHeadControllerInput(l3xz_head_ctrl::msg::HeadAngle::SharedPtr const msg);
  void updateHeadControllerInput(geometry_msgs::msg::Twist::SharedPtr const msg);
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */

#endif /* L3XZ_HEAD_CTRL_NODE_H_ */
