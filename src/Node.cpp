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
, _state{State::Init}
, _teleop_target{}
, _servo_actual{}
, _prev_ctrl_loop_timepoint{std::chrono::steady_clock::now()}
{
  declare_parameter("pan_initial_angle_deg", 180.0f);
  declare_parameter("pan_min_angle_deg", 170.0f);
  declare_parameter("pan_max_angle_deg", 190.0f);

  declare_parameter("tilt_initial_angle_deg", 180.0f);
  declare_parameter("tilt_min_angle_deg", 170.0f);
  declare_parameter("tilt_max_angle_deg", 190.0f);

  init_heartbeat();
  init_sub();
  init_pub();

  _ctrl_loop_timer = create_wall_timer(CTRL_LOOP_RATE, [this]() { this->ctrl_loop(); });

  RCLCPP_INFO(get_logger(), "%s init complete.", get_name());
}

/**************************************************************************************
 * PRIVATE MEMBER FUNCTIONS
 **************************************************************************************/

void Node::init_heartbeat()
{
  std::stringstream heartbeat_topic;
  heartbeat_topic << "/l3xz/" << get_name() << "/heartbeat";

  _heartbeat_pub = heartbeat::Publisher::create(*this, heartbeat_topic.str(), HEARTBEAT_LOOP_RATE);
}

void Node::init_sub()
{
  _head_sub = create_subscription<geometry_msgs::msg::Twist>(
    "/l3xz/cmd_vel_head", 1,
    [this](geometry_msgs::msg::Twist::SharedPtr const msg)
    {
      _teleop_target.set_angular_velocity_rps(Servo::Pan,  msg->angular.z);
      _teleop_target.set_angular_velocity_rps(Servo::Tilt, msg->angular.y);
    });

  _pan_angle_actual_sub = create_subscription<std_msgs::msg::Float32>(
    "/l3xz/head/pan/angle/actual", 1,
    [this](std_msgs::msg::Float32::SharedPtr const msg) { _servo_actual.set_angle_rad(Servo::Pan, msg->data); });

  _tilt_angle_actual_sub = create_subscription<std_msgs::msg::Float32>(
    "/l3xz/head/tilt/angle/actual", 1,
    [this](std_msgs::msg::Float32::SharedPtr const msg) { _servo_actual.set_angle_rad(Servo::Tilt, msg->data); });
}

void Node::init_pub()
{
  _pan_angle_pub      = create_publisher<std_msgs::msg::Float32>("/l3xz/head/pan/angle/target",  1);
  _pan_angle_vel_pub  = create_publisher<std_msgs::msg::Float32>("/l3xz/head/pan/angular_velocity/target",  1);

  _tilt_angle_pub     = create_publisher<std_msgs::msg::Float32>("/l3xz/head/tilt/angle/target", 1);
  _tilt_angle_vel_pub = create_publisher<std_msgs::msg::Float32>("/l3xz/head/tilt/angular_velocity/target", 1);

  _pan_angle_mode_pub  = create_publisher<ros2_dynamixel_bridge::msg::Mode>("/l3xz/head/pan/mode/set",  1);
  _tilt_angle_mode_pub = create_publisher<ros2_dynamixel_bridge::msg::Mode>("/l3xz/head/tilt/mode/set", 1);
}

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

  auto next = std::make_tuple(_state,
                              Mode::PositionControl,
                              0.0f,
                              0.0f,
                              get_parameter("pan_initial_angle_deg").as_double()  * M_PI / 180.0f,
                              get_parameter("tilt_initial_angle_deg").as_double() * M_PI / 180.0f);
  switch(_state)
  {
    case State::Init:   next = handle_Init();   break;
    case State::Teleop: next = handle_Teleop(); break;
    default:
    case State::Hold:   next = handle_Hold();   break;
  }
  auto [next_state, next_mode, next_pan_rps, next_tilt_rps, next_pan_deg, next_tilt_deg] = next;
  _state = next_state;

  publish(next_mode, next_pan_rps, next_tilt_rps, next_pan_deg, next_tilt_deg);
}

std::tuple<Node::State, Node::Mode, float, float, float, float> Node::handle_Init()
{
  float const PAN_INITIAL_ANGLE_rad  = get_parameter("pan_initial_angle_deg").as_double()  * M_PI / 180.0f;
  float const TILT_INITIAL_ANGLE_rad = get_parameter("tilt_initial_angle_deg").as_double() * M_PI / 180.0f;

  auto initial_target_angle_reached = [](float const target_rad, float const actual_rad)
  {
    static float constexpr INITIAL_ANGLE_EPSILON_rad = 2.0f * M_PI / 180.0f;
    return (fabs(target_rad - actual_rad) < INITIAL_ANGLE_EPSILON_rad);
  };

  if (initial_target_angle_reached(PAN_INITIAL_ANGLE_rad,  _servo_actual.angle_rad(Servo::Pan)) &&
      initial_target_angle_reached(TILT_INITIAL_ANGLE_rad, _servo_actual.angle_rad(Servo::Tilt)))
  {
    RCLCPP_INFO(get_logger(), "transitioning to \"State::Hold\".");
    return std::make_tuple(State::Hold, Mode::VelocityControl, 0.0f, 0.0f, _servo_actual.angle_rad(Servo::Pan), _servo_actual.angle_rad(Servo::Tilt));
  }

  return std::make_tuple(State::Init, Mode::PositionControl, 0.0f, 0.0f, PAN_INITIAL_ANGLE_rad, TILT_INITIAL_ANGLE_rad);
}

std::tuple<Node::State, Node::Mode, float, float, float, float> Node::handle_Hold()
{
  if (_teleop_target.is_active_manual_control())
  {
    RCLCPP_INFO(get_logger(), "transitioning to \"State::Teleop\" due to active manual control.");
    return std::make_tuple(State::Teleop, Mode::VelocityControl, 0.0f, 0.0f, _servo_actual.angle_rad(Servo::Pan), _servo_actual.angle_rad(Servo::Tilt));
  }

  return std::make_tuple(State::Hold, Mode::PositionControl, 0.0f, 0.0f, _servo_actual.angle_rad(Servo::Pan), _servo_actual.angle_rad(Servo::Tilt));
}

std::tuple<Node::State, Node::Mode, float, float, float, float> Node::handle_Teleop()
{
  /* Determine new target angular velocities for
   * both pan and tilt servo.
   */
  float target_pan_ang_vel_rad_per_sec  = _teleop_target.angular_velocity_rps(Servo::Pan);
  float target_tilt_ang_vel_rad_per_sec = _teleop_target.angular_velocity_rps(Servo::Tilt);

  /* Check if we are exceeding the limits and stop
   * servo movement.
   */
  static float const PAN_MIN_ANGLE_rad = get_parameter("pan_min_angle_deg").as_double() * M_PI / 180.0f;
  static float const PAN_MAX_ANGLE_rad = get_parameter("pan_max_angle_deg").as_double() * M_PI / 180.0f;

  if ((_servo_actual.angle_rad(Servo::Pan) < PAN_MIN_ANGLE_rad) && (target_pan_ang_vel_rad_per_sec < 0.0f))
    target_pan_ang_vel_rad_per_sec = 0.0f;
  if ((_servo_actual.angle_rad(Servo::Pan) > PAN_MAX_ANGLE_rad) && (target_pan_ang_vel_rad_per_sec > 0.0f))
    target_pan_ang_vel_rad_per_sec = 0.0f;

  static float const TILT_MIN_ANGLE_rad = get_parameter("tilt_min_angle_deg").as_double() * M_PI / 180.0f;
  static float const TILT_MAX_ANGLE_rad = get_parameter("tilt_max_angle_deg").as_double() * M_PI / 180.0f;

  if ((_servo_actual.angle_rad(Servo::Tilt) < TILT_MIN_ANGLE_rad) && (target_tilt_ang_vel_rad_per_sec < 0.0f))
    target_tilt_ang_vel_rad_per_sec = 0.0f;
  if ((_servo_actual.angle_rad(Servo::Tilt) > TILT_MAX_ANGLE_rad) && (target_tilt_ang_vel_rad_per_sec > 0.0f))
    target_tilt_ang_vel_rad_per_sec = 0.0f;

  /* Update the activity time-point, if we are currently actively
   * teleoperating the robots sensor head.
   */
  auto const now = std::chrono::steady_clock::now();

  if (_teleop_target.is_active_manual_control())
    _prev_teleop_activity_timepoint = now;

  if ((now - _prev_teleop_activity_timepoint) > std::chrono::seconds(5))
  {
    RCLCPP_INFO(get_logger(), "transitioning to \"State::Hold\" due to inactivity timeout on manual control.");
    return std::make_tuple(State::Hold, Mode::PositionControl, 0.0f, 0.0f, _servo_actual.angle_rad(Servo::Pan), _servo_actual.angle_rad(Servo::Tilt));
  }

  return std::make_tuple(State::Teleop, Mode::VelocityControl, target_pan_ang_vel_rad_per_sec, target_tilt_ang_vel_rad_per_sec, _servo_actual.angle_rad(Servo::Pan), _servo_actual.angle_rad(Servo::Tilt));
}

void Node::publish(Mode const mode, float const pan_rps, float const tilt_rps, float const pan_deg, float const tilt_deg)
{
  if (mode == Mode::PositionControl)
  {
    publish_mode_PositionControl(_pan_angle_mode_pub);
    publish_mode_PositionControl(_tilt_angle_mode_pub);
  }
  else
  {
    publish_mode_VelocityControl(_pan_angle_mode_pub);
    publish_mode_VelocityControl(_tilt_angle_mode_pub);
  }

  publish_AngularVelocity(_pan_angle_vel_pub, pan_rps);
  publish_AngularVelocity(_tilt_angle_vel_pub, tilt_rps);

  publish_Angle(_pan_angle_pub,  pan_deg);
  publish_Angle(_tilt_angle_pub, tilt_deg);
}

void Node::publish_AngularVelocity(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr const pub, float const angular_velocity_rad_per_sec)
{
  std_msgs::msg::Float32 msg;
  msg.data = angular_velocity_rad_per_sec;
  pub->publish(msg);
}

void Node::publish_Angle(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr const pub, float const angle_rad)
{
  std_msgs::msg::Float32 msg;
  msg.data = angle_rad;
  pub->publish(msg);
}

void Node::publish_mode_PositionControl(rclcpp::Publisher<ros2_dynamixel_bridge::msg::Mode>::SharedPtr const pub)
{
  ros2_dynamixel_bridge::msg::Mode msg;
  msg.servo_mode = ros2_dynamixel_bridge::msg::Mode::SERVO_MODE_POSITION_CONTROL;
  pub->publish(msg);
}

void Node::publish_mode_VelocityControl(rclcpp::Publisher<ros2_dynamixel_bridge::msg::Mode>::SharedPtr const pub)
{
  ros2_dynamixel_bridge::msg::Mode msg;
  msg.servo_mode = ros2_dynamixel_bridge::msg::Mode::SERVO_MODE_VELOCITY_CONTROL;
  pub->publish(msg);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */
