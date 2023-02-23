/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_head_ctrl/Node.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Node::Node()
: rclcpp::Node("l3xz_head_ctrl")
, _pan_angular_velocity_rad_per_sec{0.0f}
, _tilt_angular_velocity_rad_per_sec{0.0f}
, _pan_angle_rad_actual{0.0f}
, _tilt_angle_rad_actual{0.0f}
, _pan_angle_rad_target{0.0f}
, _tilt_angle_rad_target{0.0f}
, _state{State::Init}
, _prev_ctrl_loop_timepoint{std::chrono::steady_clock::now()}
{
  declare_parameter("pan_initial_angle_deg", 180.0f);
  declare_parameter("pan_min_angle_deg", 170.0f);
  declare_parameter("pan_max_angle_deg", 190.0f);

  declare_parameter("tilt_initial_angle_deg", 180.0f);
  declare_parameter("tilt_min_angle_deg", 170.0f);
  declare_parameter("tilt_max_angle_deg", 190.0f);

  /* Configure subscribers and publishers. */
  _head_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel_head", 1,
    [this](geometry_msgs::msg::Twist::SharedPtr const msg)
    {
      _pan_angular_velocity_rad_per_sec  = msg->angular.z;
      _tilt_angular_velocity_rad_per_sec = msg->angular.y;
    });

  _pan_angle_actual_sub = create_subscription<std_msgs::msg::Float32>
    ("/l3xz/head/pan/angle/actual", 1,
     [this](std_msgs::msg::Float32::SharedPtr const msg) { _pan_angle_rad_actual = msg->data; });

  _tilt_angle_actual_sub = create_subscription<std_msgs::msg::Float32>
    ("/l3xz/head/tilt/angle/actual", 1,
     [this](std_msgs::msg::Float32::SharedPtr const msg) { _tilt_angle_rad_actual = msg->data; });

  _pan_angle_pub      = create_publisher<std_msgs::msg::Float32>("/l3xz/head/pan/angle/target",  1);
  _pan_angle_vel_pub  = create_publisher<std_msgs::msg::Float32>("/l3xz/head/pan/angular_velocity/target",  1);

  _tilt_angle_pub     = create_publisher<std_msgs::msg::Float32>("/l3xz/head/tilt/angle/target", 1);
  _tilt_angle_vel_pub = create_publisher<std_msgs::msg::Float32>("/l3xz/head/tilt/angular_velocity/target", 1);

  _pan_angle_mode_pub  = create_publisher<ros2_dynamixel_bridge::msg::Mode>("/l3xz/head/pan/mode/set",  1);
  _tilt_angle_mode_pub = create_publisher<ros2_dynamixel_bridge::msg::Mode>("/l3xz/head/tilt/mode/set", 1);

  /* Configure periodic control loop function. */
  _ctrl_loop_timer = create_wall_timer
    (std::chrono::milliseconds(CTRL_LOOP_RATE.count()),
     [this]() { this->ctrl_loop(); });

  RCLCPP_INFO(get_logger(), "node initialization complete.");
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::ctrl_loop()
{
  auto const now = std::chrono::steady_clock::now();
  auto const ctrl_loop_rate = (now - _prev_ctrl_loop_timepoint);
  if (ctrl_loop_rate > (CTRL_LOOP_RATE + std::chrono::milliseconds(1)))
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "ctrl_loop should be called every %ld ms, but is %ld ms instead",
                         CTRL_LOOP_RATE.count(),
                         std::chrono::duration_cast<std::chrono::milliseconds>(ctrl_loop_rate).count());
  _prev_ctrl_loop_timepoint = now;

  State next_state = _state;
  switch(_state)
  {
    case State::Init:   next_state = handle_Init();   break;
    case State::Teleop: next_state = handle_Teleop(); break;
    default:
    case State::Hold:   next_state = handle_Hold();   break;
  }
  _state = next_state;
}

Node::State Node::handle_Init()
{
  setMode_PositionControl(_pan_angle_mode_pub);
  setMode_PositionControl(_tilt_angle_mode_pub);

  float const PAN_INITIAL_ANGLE_rad  = get_parameter("pan_initial_angle_deg").as_double()  * M_PI / 180.0f;
  float const TILT_INITIAL_ANGLE_rad = get_parameter("tilt_initial_angle_deg").as_double() * M_PI / 180.0f;

  setAngle(_pan_angle_pub,  PAN_INITIAL_ANGLE_rad);
  setAngle(_tilt_angle_pub, TILT_INITIAL_ANGLE_rad);

  auto initial_target_angle_reached = [](float const target_rad, float const actual_rad)
  {
    static float constexpr INITIAL_ANGLE_EPSILON_rad = 2.0f * M_PI / 180.0f;
    return (fabs(target_rad - actual_rad) < INITIAL_ANGLE_EPSILON_rad);
  };

  if (initial_target_angle_reached(PAN_INITIAL_ANGLE_rad, _pan_angle_rad_actual) &&
      initial_target_angle_reached(TILT_INITIAL_ANGLE_rad, _tilt_angle_rad_actual))
  {
    RCLCPP_INFO(get_logger(), "transitioning to \"State::Hold\".");
    _pan_angle_rad_target = _pan_angle_rad_actual;
    _tilt_angle_rad_target = _tilt_angle_rad_actual;
    return State::Hold;
  }

  return State::Init;
}

Node::State Node::handle_Hold()
{
  setMode_PositionControl(_pan_angle_mode_pub);
  setMode_PositionControl(_tilt_angle_mode_pub);

  setAngle(_pan_angle_pub, _pan_angle_rad_target);
  setAngle(_tilt_angle_pub, _tilt_angle_rad_target);

  if (fabs(_pan_angular_velocity_rad_per_sec)  > ACTIVITY_EPSILON_rad_per_sec ||
      fabs(_tilt_angular_velocity_rad_per_sec) > ACTIVITY_EPSILON_rad_per_sec)
  {
    RCLCPP_INFO(get_logger(), "transitioning to \"State::Teleop\" due to active manual control.");
    return State::Teleop;
  }

  return State::Hold;
}

Node::State Node::handle_Teleop()
{
  setMode_VelocityControl(_pan_angle_mode_pub);
  setMode_VelocityControl(_tilt_angle_mode_pub);

  /* Determine new target angular velocities for
   * both pan and tilt servo.
   */
  float target_pan_ang_vel_rad_per_sec  = _pan_angular_velocity_rad_per_sec;
  float target_tilt_ang_vel_rad_per_sec = _tilt_angular_velocity_rad_per_sec;

  /* Check if we are exceeding the limits and stop
   * servo movement.
   */
  static float const PAN_MIN_ANGLE_rad = get_parameter("pan_min_angle_deg").as_double() * M_PI / 180.0f;
  static float const PAN_MAX_ANGLE_rad = get_parameter("pan_max_angle_deg").as_double() * M_PI / 180.0f;

  if ((_pan_angle_rad_actual < PAN_MIN_ANGLE_rad) && (target_pan_ang_vel_rad_per_sec < 0.0f))
    target_pan_ang_vel_rad_per_sec = 0.0f;
  if ((_pan_angle_rad_actual > PAN_MAX_ANGLE_rad) && (target_pan_ang_vel_rad_per_sec > 0.0f))
    target_pan_ang_vel_rad_per_sec = 0.0f;

  static float const TILT_MIN_ANGLE_rad = get_parameter("tilt_min_angle_deg").as_double() * M_PI / 180.0f;
  static float const TILT_MAX_ANGLE_rad = get_parameter("tilt_max_angle_deg").as_double() * M_PI / 180.0f;

  if ((_tilt_angle_rad_actual < TILT_MIN_ANGLE_rad) && (target_tilt_ang_vel_rad_per_sec < 0.0f))
    target_tilt_ang_vel_rad_per_sec = 0.0f;
  if ((_tilt_angle_rad_actual > TILT_MAX_ANGLE_rad) && (target_tilt_ang_vel_rad_per_sec > 0.0f))
    target_tilt_ang_vel_rad_per_sec = 0.0f;

  /* Publish the desired angular target velocities to
   * the ROS2/Dynamixel bridge.
   */
  setAngularVelocity(_pan_angle_vel_pub, target_pan_ang_vel_rad_per_sec);
  setAngularVelocity(_tilt_angle_vel_pub, target_tilt_ang_vel_rad_per_sec);

  /* Update the activity time-point, if we are currently actively
   * teleoperating the robots sensor head.
   */
  auto const now = std::chrono::steady_clock::now();

  if (fabs(_pan_angular_velocity_rad_per_sec)  > ACTIVITY_EPSILON_rad_per_sec ||
      fabs(_tilt_angular_velocity_rad_per_sec) > ACTIVITY_EPSILON_rad_per_sec) {
    _prev_teleop_activity_timepoint = now;
  }

  if ((now - _prev_teleop_activity_timepoint) > std::chrono::seconds(5))
  {
    RCLCPP_INFO(get_logger(), "transitioning to \"State::Hold\" due to inactivity timeout on manual control.");
    _pan_angle_rad_target = _pan_angle_rad_actual;
    _tilt_angle_rad_target = _tilt_angle_rad_actual;
    return State::Hold;
  }

  return State::Teleop;
}

void Node::setAngularVelocity(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr const pub, float const angular_velocity_rad_per_sec)
{
  std_msgs::msg::Float32 msg;
  msg.data = angular_velocity_rad_per_sec;
  pub->publish(msg);
}

void Node::setAngle(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr const pub, float const angle_rad)
{
  std_msgs::msg::Float32 msg;
  msg.data = angle_rad;
  pub->publish(msg);
}

void Node::setMode_PositionControl(rclcpp::Publisher<ros2_dynamixel_bridge::msg::Mode>::SharedPtr const pub)
{
  ros2_dynamixel_bridge::msg::Mode msg;
  msg.servo_mode = ros2_dynamixel_bridge::msg::Mode::SERVO_MODE_POSITION_CONTROL;
  pub->publish(msg);
}

void Node::setMode_VelocityControl(rclcpp::Publisher<ros2_dynamixel_bridge::msg::Mode>::SharedPtr const pub)
{
  ros2_dynamixel_bridge::msg::Mode msg;
  msg.servo_mode = ros2_dynamixel_bridge::msg::Mode::SERVO_MODE_VELOCITY_CONTROL;
  pub->publish(msg);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */
