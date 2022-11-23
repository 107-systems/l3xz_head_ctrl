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
, _pan_servo_id{DEFAULT_PAN_SERVO_ID}
, _tilt_servo_id{DEFAULT_TILT_SERVO_ID}
{
  /* Configure the Dynamixel MX-28AR servos of the pan/tilt head. */

  declare_parameter("serial_port", "/dev/ttyUSB0");
  declare_parameter("serial_port_baudrate", DEFAULT_SERIAL_BAUDRATE);
  declare_parameter("pan_servo_id", DEFAULT_PAN_SERVO_ID);
  declare_parameter("tilt_servo_id", DEFAULT_TILT_SERVO_ID);

  std::string const serial_port  = get_parameter("serial_port").as_string();
  int const serial_port_baudrate = get_parameter("serial_port_baudrate").as_int();

  _dyn_ctrl.reset(new dynamixelplusplus::Dynamixel(serial_port, dynamixelplusplus::Dynamixel::Protocol::V2_0, serial_port_baudrate));

  /* Determine which/if any servos can be reached via the connected network. */
  auto [err_ping, dyn_id_vect] = _dyn_ctrl->broadcastPing();
  if (err_ping != dynamixelplusplus::Dynamixel::Error::None) {
    RCLCPP_ERROR(get_logger(), "'broadcastPing' failed with error code %d", static_cast<int>(err_ping));
    rclcpp::shutdown();
  }

  std::stringstream dyn_id_list;
  for (auto id : dyn_id_vect)
    dyn_id_list << static_cast<int>(id) << " ";
  RCLCPP_INFO(get_logger(), "detected Dynamixel MX-28: { %s}", dyn_id_list.str().c_str());

  _pan_servo_id  = static_cast<dynamixelplusplus::Dynamixel::Id>(get_parameter("pan_servo_id").as_int());
  _tilt_servo_id = static_cast<dynamixelplusplus::Dynamixel::Id>(get_parameter("tilt_servo_id").as_int());

  if (std::none_of(std::cbegin(dyn_id_vect), std::cend(dyn_id_vect), [this](dynamixelplusplus::Dynamixel::Id const id) { return (id == _pan_servo_id); })) {
    RCLCPP_ERROR(get_logger(), "pan servo with configured id %d not online", static_cast<int>(_pan_servo_id));
    rclcpp::shutdown();
  }

  if (std::none_of(std::cbegin(dyn_id_vect), std::cend(dyn_id_vect), [this](dynamixelplusplus::Dynamixel::Id const id) { return (id == _tilt_servo_id); })) {
    RCLCPP_ERROR(get_logger(), "tilt servo with configured id %d not online", static_cast<int>(_tilt_servo_id));
    rclcpp::shutdown();
  }

  /* Configure subscribers and publishers. */

  _head_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel_head", 10, [this](geometry_msgs::msg::Twist::SharedPtr const msg) { updateHeadControllerInput(msg);});

  _head_angle_pub = create_publisher<l3xz_head_ctrl::msg::HeadAngle>
    ("/l3xz/ctrl/head/angle/target", 10);

  _head_angle_sub = create_subscription<l3xz_head_ctrl::msg::HeadAngle>
    ("/l3xz/ctrl/head/angle/actual", 10, [this](l3xz_head_ctrl::msg::HeadAngle::SharedPtr const msg) { updateHeadControllerInput(msg); });

  /* Configure periodic control loop function. */

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
  _head_ctrl_input.set_pan_angular_velocity (msg->angular.z);
  _head_ctrl_input.set_tilt_angular_velocity(msg->angular.y);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
