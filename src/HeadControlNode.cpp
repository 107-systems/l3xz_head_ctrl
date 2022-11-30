/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
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

using namespace dynamixelplusplus;
using namespace mx28ar;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

HeadControlNode::HeadControlNode()
: rclcpp::Node("l3xz_head_ctrl")
, _mx28_ctrl{}
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
  declare_parameter("pan_servo_initial_angle", DEFAULT_PAN_SERVO_INITIAL_ANGLE);
  declare_parameter("tilt_servo_initial_angle", DEFAULT_TILT_SERVO_INITIAL_ANGLE);

  std::string const serial_port  = get_parameter("serial_port").as_string();
  int const serial_port_baudrate = get_parameter("serial_port_baudrate").as_int();

  RCLCPP_INFO(get_logger(), "configuring Dynamixel RS485 bus:\n\tDevice:   %s\n\tBaudrate: %d", serial_port.c_str(), serial_port_baudrate);

  std::unique_ptr<Dynamixel> dyn_ctrl(new Dynamixel(serial_port, Dynamixel::Protocol::V2_0, serial_port_baudrate));

  /* Determine which/if any servos can be reached via the connected network. */
  auto [err_ping, dyn_id_vect] = dyn_ctrl->broadcastPing();
  if (err_ping != Dynamixel::Error::None) {
    RCLCPP_ERROR(get_logger(), "'broadcastPing' failed with error code %d", static_cast<int>(err_ping));
    rclcpp::shutdown();
  }

  std::stringstream dyn_id_list;
  for (auto id : dyn_id_vect)
    dyn_id_list << static_cast<int>(id) << " ";
  RCLCPP_INFO(get_logger(), "detected Dynamixel MX-28AR: { %s}", dyn_id_list.str().c_str());

  _pan_servo_id  = static_cast<Dynamixel::Id>(get_parameter("pan_servo_id").as_int());
  _tilt_servo_id = static_cast<Dynamixel::Id>(get_parameter("tilt_servo_id").as_int());

  if (std::none_of(std::cbegin(dyn_id_vect), std::cend(dyn_id_vect), [this](Dynamixel::Id const id) { return (id == _pan_servo_id); })) {
    RCLCPP_ERROR(get_logger(), "pan servo with configured id %d not online", static_cast<int>(_pan_servo_id));
    rclcpp::shutdown();
  }

  if (std::none_of(std::cbegin(dyn_id_vect), std::cend(dyn_id_vect), [this](Dynamixel::Id const id) { return (id == _tilt_servo_id); })) {
    RCLCPP_ERROR(get_logger(), "tilt servo with configured id %d not online", static_cast<int>(_tilt_servo_id));
    rclcpp::shutdown();
  }

  /* Instantiate MX-28AR controller and continue with pan/tilt head initialization. */
  _mx28_ctrl.reset(new MX28AR_Control(std::move(dyn_ctrl)));

  Dynamixel::IdVect const pan_tilt_id_vect{_pan_servo_id, _tilt_servo_id};

  if (!_mx28_ctrl->setTorqueEnable(pan_tilt_id_vect, TorqueEnable::Off)) {
    RCLCPP_ERROR(get_logger(), "could not disable torque for pan/tilt servos");
    rclcpp::shutdown();
  }

  if (!_mx28_ctrl->setOperatingMode(pan_tilt_id_vect, OperatingMode::PositionControlMode)) {
    RCLCPP_ERROR(get_logger(), "could not configure pan/tilt servos for position control mode");
    rclcpp::shutdown();
  }

  std::map<Dynamixel::Id, float> const INITIAL_HEAD_POSITION_deg =
  {
    {_pan_servo_id, get_parameter("pan_servo_initial_angle").as_double()},
    {_tilt_servo_id, get_parameter("tilt_servo_initial_angle").as_double()}
  };
  if (!_mx28_ctrl->setGoalPosition(INITIAL_HEAD_POSITION_deg)) {
    RCLCPP_ERROR(get_logger(),
                 "could not set initial position for pan (%0.2f) / tilt (%0.2f) servo",
                 INITIAL_HEAD_POSITION_deg.at(_pan_servo_id),
                 INITIAL_HEAD_POSITION_deg.at(_tilt_servo_id));
    rclcpp::shutdown();
  }

  if (!_mx28_ctrl->setTorqueEnable(pan_tilt_id_vect, TorqueEnable::On)) {
    RCLCPP_ERROR(get_logger(), "could not enable torque for pan/tilt servos");
    rclcpp::shutdown();
  }

  /* Configure subscribers and publishers. */

  _head_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel_head", 10,
    [this](geometry_msgs::msg::Twist::SharedPtr const msg)
    {
      _head_ctrl_input.set_pan_angular_velocity (msg->angular.z);
      _head_ctrl_input.set_tilt_angular_velocity(msg->angular.y);
    });

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
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz */
