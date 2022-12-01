/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_head_ctrl/Node.h>

#include <l3xz_head_ctrl/MX28AR/MX28AR_Control.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head
{

using namespace dynamixelplusplus;
using namespace mx28ar;

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define CHECK(cond,...)                      \
  if (cond) {                                \
    RCLCPP_ERROR(get_logger(), __VA_ARGS__); \
    rclcpp::shutdown();                      \
  }

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_head_ctrl")
, _head_ctrl{}
, _pan_servo_id{DEFAULT_PAN_SERVO_ID}
, _tilt_servo_id{DEFAULT_TILT_SERVO_ID}
, _pan_angular_velocity_dps{0.0f}
, _tilt_angular_velocity_dps{0.0f}
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
  CHECK(err_ping != Dynamixel::Error::None, "'broadcastPing' failed with error code %d", static_cast<int>(err_ping));

  std::stringstream dyn_id_list;
  for (auto id : dyn_id_vect)
    dyn_id_list << static_cast<int>(id) << " ";
  RCLCPP_INFO(get_logger(), "detected Dynamixel MX-28AR: { %s}.", dyn_id_list.str().c_str());

  _pan_servo_id  = static_cast<Dynamixel::Id>(get_parameter("pan_servo_id").as_int());
  _tilt_servo_id = static_cast<Dynamixel::Id>(get_parameter("tilt_servo_id").as_int());

  CHECK(std::none_of(std::cbegin(dyn_id_vect), std::cend(dyn_id_vect), [this](Dynamixel::Id const id) { return (id == _pan_servo_id); }),
        "pan servo with configured id %d not online.", static_cast<int>(_pan_servo_id));
  CHECK(std::none_of(std::cbegin(dyn_id_vect), std::cend(dyn_id_vect), [this](Dynamixel::Id const id) { return (id == _tilt_servo_id); }),
        "tilt servo with configured id %d not online.", static_cast<int>(_tilt_servo_id));

  /* Instantiate MX-28AR controller and continue with pan/tilt head initialization. */
  std::unique_ptr<MX28AR_Control> mx28_ctrl(new MX28AR_Control(std::move(dyn_ctrl)));

  RCLCPP_INFO(get_logger(), "initialize pan/servo in position control mode and set to initial angle.");

  Dynamixel::IdVect const pan_tilt_id_vect{_pan_servo_id, _tilt_servo_id};

  CHECK(!mx28_ctrl->setTorqueEnable(pan_tilt_id_vect, TorqueEnable::Off), "could not disable torque for pan/tilt servos.");
  CHECK(!mx28_ctrl->setOperatingMode(pan_tilt_id_vect, OperatingMode::PositionControlMode), "could not configure pan/tilt servos for position control mode.");
  CHECK(!mx28_ctrl->setTorqueEnable(pan_tilt_id_vect, TorqueEnable::On), "could not enable torque for pan/tilt servos.");

  std::map<Dynamixel::Id, float> const INITIAL_HEAD_POSITION_deg =
  {
    {_pan_servo_id, get_parameter("pan_servo_initial_angle").as_double()},
    {_tilt_servo_id, get_parameter("tilt_servo_initial_angle").as_double()}
  };
  CHECK(!mx28_ctrl->setGoalPosition(INITIAL_HEAD_POSITION_deg),
        "could not set initial position for pan (%0.2f) / tilt (%0.2f) servo.",
        INITIAL_HEAD_POSITION_deg.at(_pan_servo_id),
        INITIAL_HEAD_POSITION_deg.at(_tilt_servo_id));

  bool pan_target_reached = false, tilt_target_reached = false;
  std::map<Dynamixel::Id, float> actual_head_position_deg;
  for (auto const start = std::chrono::system_clock::now();
       (std::chrono::system_clock::now() - start) < std::chrono::seconds(5) && !pan_target_reached && !tilt_target_reached; )
  {
    std::this_thread::sleep_for(std::chrono::seconds(1));

    CHECK(!mx28_ctrl->getPresentPosition(pan_tilt_id_vect, actual_head_position_deg), "could not read current position for pan/tilt servo.");
    CHECK(!actual_head_position_deg.count(_pan_servo_id), "no position data for pan servo.");
    CHECK(!actual_head_position_deg.count(_tilt_servo_id), "no position data for tilt servo.");

    static float constexpr INITIAL_ANGLE_EPSILON_deg = 2.0f;
    pan_target_reached  = fabs(actual_head_position_deg.at(_pan_servo_id)  - INITIAL_HEAD_POSITION_deg.at(_pan_servo_id))  < INITIAL_ANGLE_EPSILON_deg;
    tilt_target_reached = fabs(actual_head_position_deg.at(_tilt_servo_id) - INITIAL_HEAD_POSITION_deg.at(_tilt_servo_id)) < INITIAL_ANGLE_EPSILON_deg;
  }
  CHECK(!pan_target_reached,
        "could not reach initial position for pan servo, target: %0.2f, actual: %0.2f.",
        INITIAL_HEAD_POSITION_deg.at(_pan_servo_id),
        actual_head_position_deg.at(_pan_servo_id));
  CHECK(!tilt_target_reached,
        "could not reach initial position for tilt servo, target: %0.2f, actual: %0.2f.",
        INITIAL_HEAD_POSITION_deg.at(_tilt_servo_id),
        actual_head_position_deg.at(_tilt_servo_id));


  _head_ctrl.reset(new Controller(std::move(mx28_ctrl), get_logger(), _pan_servo_id, _tilt_servo_id));

  /* Configure subscribers and publishers. */

  _head_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel_head", 10,
    [this](geometry_msgs::msg::Twist::SharedPtr const msg)
    {
      _pan_angular_velocity_dps  = msg->angular.z;
      _tilt_angular_velocity_dps = msg->angular.y;
    });

  /* Configure periodic control loop function. */

  _ctrl_loop_timer = create_wall_timer
    (std::chrono::milliseconds(50),
     [this]()
     {
       _head_ctrl->update(_pan_angular_velocity_dps, _tilt_angular_velocity_dps);
     });

  RCLCPP_INFO(get_logger(), "node initialization complete.");
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */
