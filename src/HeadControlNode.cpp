/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_head_ctrl/HeadControlNode.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

HeadControlNode::HeadControlNode()
: rclcpp::Node("l3xz_head_ctrl")
, _head_ctrl{}
, _head_ctrl_input{}
, _head_ctrl_output{}
{
  _head_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel_head", 10, [this](geometry_msgs::msg::Twist::SharedPtr const msg) { updateHeadControllerInput(msg);});

  _head_angle_pub = create_publisher<l3xz_head_ctrl::msg::HeadAngle>
    ("/l3xz/ctrl/head/angle/target", 10);

  _head_angle_sub = create_subscription<l3xz_head_ctrl::msg::HeadAngle>
    ("/l3xz/ctrl/head/angle/actual", 10, [this](l3xz_head_ctrl::msg::HeadAngle::SharedPtr const msg) { updateHeadControllerInput(msg); });

  _ctrl_loop_timer = create_wall_timer
    (std::chrono::milliseconds(50), [this]() { this->onCtrlLoopTimerEvent(); });
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void HeadControlNode::onCtrlLoopTimerEvent()
{
  _head_ctrl_output = _head_ctrl.update(_head_ctrl_input, _head_ctrl_output);

  l3xz_head_ctrl::msg::HeadAngle head_msg;
  head_msg.pan_angle_deg  = _head_ctrl_output.pan_angle ();
  head_msg.tilt_angle_deg = _head_ctrl_output.tilt_angle();
  _head_angle_pub->publish(head_msg);
}

void HeadControlNode::updateHeadControllerInput(l3xz_head_ctrl::msg::HeadAngle::SharedPtr msg)
{
  _head_ctrl_input.set_pan_angle (msg->pan_angle_deg);
  _head_ctrl_input.set_tilt_angle(msg->tilt_angle_deg);
}

void HeadControlNode::updateHeadControllerInput(geometry_msgs::msg::Twist::SharedPtr const msg)
{
  _head_ctrl_input.set_pan_angular_velocity (msg->angular.y);
  _head_ctrl_input.set_tilt_angular_velocity(msg->angular.z);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
