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
, _head_qos_profile
{
  rclcpp::KeepLast(10),
  rmw_qos_profile_sensor_data
}
, _target_angular_velocity
{
  {Servo::Pan,  0. * rad/s},
  {Servo::Tilt, 0. * rad/s},
}
, _actual_angle
{
  {Servo::Pan,  0. * rad},
  {Servo::Tilt, 0. * rad},
}
, _target_angle
{
  {Servo::Pan,  0. * rad},
  {Servo::Tilt, 0. * rad},
}
, _target_mode{Mode::PositionControl}
{
  declare_parameter("pan_initial_angle_deg", 180.0f);
  declare_parameter("pan_min_angle_deg", 170.0f);
  declare_parameter("pan_max_angle_deg", 190.0f);

  declare_parameter("tilt_initial_angle_deg", 90.0f);
  declare_parameter("tilt_min_angle_deg", 80.0f);
  declare_parameter("tilt_max_angle_deg", 100.0f);

  init_heartbeat();
  init_head_sub();
  init_actual_angle();
  init_pub();

  _ctrl_loop_rate_monitor = loop_rate::Monitor::create(CTRL_LOOP_RATE, std::chrono::milliseconds(1));
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

  _heartbeat_pub = heartbeat::Publisher::create(*this, heartbeat_topic.str());
}

void Node::init_head_sub()
{
  declare_parameter("head_topic", "cmd_vel_head");
  declare_parameter("head_topic_deadline_ms", 100);
  declare_parameter("head_topic_liveliness_lease_duration", 1000);

  auto const head_topic = get_parameter("head_topic").as_string();
  auto const head_topic_deadline = std::chrono::milliseconds(get_parameter("head_topic_deadline_ms").as_int());
  auto const head_topic_liveliness_lease_duration = std::chrono::milliseconds(get_parameter("head_topic_liveliness_lease_duration").as_int());

  _head_qos_profile.deadline(head_topic_deadline);
  _head_qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_MANUAL_BY_TOPIC);
  _head_qos_profile.liveliness_lease_duration(head_topic_liveliness_lease_duration);

  _head_sub_options.event_callbacks.deadline_callback =
    [this, head_topic](rclcpp::QOSDeadlineRequestedInfo & event) -> void
    {
      RCLCPP_ERROR_THROTTLE(get_logger(),
                            *get_clock(),
                            5*1000UL,
                            "deadline missed for \"%s\" (total_count: %d, total_count_change: %d).",
                            head_topic.c_str(),
                            event.total_count,
                            event.total_count_change);

      _target_angular_velocity[Servo::Pan ] = 0. * rad/s;
      _target_angular_velocity[Servo::Tilt] = 0. * rad/s;
    };

  _head_sub_options.event_callbacks.liveliness_callback =
    [this, head_topic](rclcpp::QOSLivelinessChangedInfo & event) -> void
    {
      if (event.alive_count > 0)
      {
        RCLCPP_INFO(get_logger(), "liveliness gained for \"%s\"", head_topic.c_str());
      }
      else
      {
        RCLCPP_WARN(get_logger(), "liveliness lost for \"%s\"", head_topic.c_str());

        _target_angular_velocity[Servo::Pan ] = 0. * rad/s;
        _target_angular_velocity[Servo::Tilt] = 0. * rad/s;
      }
    };

  _head_sub = create_subscription<geometry_msgs::msg::Twist>(
    head_topic,
    _head_qos_profile,
    [this](geometry_msgs::msg::Twist::SharedPtr const msg)
    {
      _target_angular_velocity[Servo::Pan ] = static_cast<double>(msg->angular.z) * rad/s;
      _target_angular_velocity[Servo::Tilt] = static_cast<double>(msg->angular.y) * rad/s;
    },
    _head_sub_options);
}

void Node::init_actual_angle()
{
  _actual_angle_sub[Servo::Pan] = create_subscription<std_msgs::msg::Float32>(
    "/l3xz/head/pan/angle/actual", 1,
    [this](std_msgs::msg::Float32::SharedPtr const msg)
    {
      _actual_angle[Servo::Pan] = static_cast<double>(msg->data) * rad;
    });

  _actual_angle_sub[Servo::Tilt] = create_subscription<std_msgs::msg::Float32>(
    "/l3xz/head/tilt/angle/actual", 1,
    [this](std_msgs::msg::Float32::SharedPtr const msg)
    {
      _actual_angle[Servo::Tilt] = static_cast<double>(msg->data) * rad;
    });
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
  _ctrl_loop_rate_monitor->update();
  if (auto const [timeout, opt_timeout_duration] = _ctrl_loop_rate_monitor->isTimeout();
    timeout == loop_rate::Monitor::Timeout::Yes)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         1000,
                         "ctrl_loop should be called every %ld ms, but is %ld ms instead",
                         CTRL_LOOP_RATE.count(),
                         opt_timeout_duration.value().count());
  }

  auto next_state = _state;

  switch(_state)
  {
    case State::Init:    next_state = handle_Init();    break;
    case State::Startup: next_state = handle_Startup(); break;
    case State::Teleop:  next_state = handle_Teleop();  break;
    default:
    case State::Hold:    next_state = handle_Hold();    break;
  }
  _state = next_state;

  publish();
}

Node::State Node::handle_Init()
{
  bool all_messages_received = true;
  std::stringstream missing_msg_list;

  if (!all_messages_received)
  {
    RCLCPP_WARN_THROTTLE(get_logger(),
                         *get_clock(),
                         2000,
                         "missing messages for topics [ %s]",
                         missing_msg_list.str().c_str());

    return State::Init;
  }

  /* We have valid messages from all topics, let's get active. */
  RCLCPP_INFO(get_logger(), "State::Init -> State::Startup");
  return State::Startup;
}

Node::State Node::handle_Startup()
{
  auto const PAN_INITIAL_ANGLE  = (get_parameter("pan_initial_angle_deg").as_double()  * deg).in(rad);
  auto const TILT_INITIAL_ANGLE = (get_parameter("tilt_initial_angle_deg").as_double() * deg).in(rad);

  auto initial_target_angle_reached = [](quantity<rad> const target, quantity<rad> const actual)
  {
    static auto constexpr INITIAL_ANGLE_EPSILON = (2. * deg).in(rad);
    auto const diff = target - actual;
    return (diff > -1. * INITIAL_ANGLE_EPSILON &&
            diff <       INITIAL_ANGLE_EPSILON);
  };

  if (initial_target_angle_reached(PAN_INITIAL_ANGLE,  _actual_angle.at(Servo::Pan)) &&
      initial_target_angle_reached(TILT_INITIAL_ANGLE, _actual_angle.at(Servo::Tilt)))
  {
    RCLCPP_INFO(get_logger(), "State::Startup -> State::Hold");

    _target_angle[Servo::Pan ] = _actual_angle.at(Servo::Pan);
    _target_angle[Servo::Tilt] = _actual_angle.at(Servo::Tilt);

    return State::Hold;
  }

  return State::Startup;
}

Node::State Node::handle_Hold()
{
  if (is_active_manual_control())
  {
    RCLCPP_INFO(get_logger(), "State::Hold -> State::Teleop due to active manual control.");

    _target_mode = Mode::VelocityControl;

    return State::Teleop;
  }

  return State::Hold;
}

Node::State Node::handle_Teleop()
{
  /* Check if we are exceeding the limits and stop
   * servo movement.
   */
  static auto const PAN_MIN_ANGLE = (get_parameter("pan_min_angle_deg").as_double() * deg).in(rad);
  static auto const PAN_MAX_ANGLE = (get_parameter("pan_max_angle_deg").as_double() * deg).in(rad);

  if ((_actual_angle.at(Servo::Pan) < PAN_MIN_ANGLE) && (_target_angular_velocity.at(Servo::Pan) < 0. * rad/s))
    _target_angular_velocity[Servo::Pan] = 0. * rad/s;
  if ((_actual_angle.at(Servo::Pan) > PAN_MAX_ANGLE) && (_target_angular_velocity.at(Servo::Pan) > 0. * rad/s))
    _target_angular_velocity[Servo::Pan] = 0. * rad/s;

  static auto const TILT_MIN_ANGLE = (get_parameter("tilt_min_angle_deg").as_double() * deg).in(rad);
  static auto const TILT_MAX_ANGLE = (get_parameter("tilt_max_angle_deg").as_double() * deg).in(rad);

  if ((_actual_angle.at(Servo::Tilt) < TILT_MIN_ANGLE) && (_target_angular_velocity.at(Servo::Tilt) < 0. * rad/s))
    _target_angular_velocity[Servo::Tilt] = 0. * rad/s;
  if ((_actual_angle.at(Servo::Tilt) > TILT_MAX_ANGLE) && (_target_angular_velocity.at(Servo::Tilt) > 0. * rad/s))
    _target_angular_velocity[Servo::Tilt] = 0. * rad/s;

  /* Update the activity time-point, if we are currently actively
   * teleoperating the robots sensor head.
   */
  auto const now = std::chrono::steady_clock::now();

  if (is_active_manual_control())
    _prev_teleop_activity_timepoint = now;

  if ((now - _prev_teleop_activity_timepoint) > std::chrono::seconds(5))
  {
    RCLCPP_INFO(get_logger(), "State::Teleop -> State::Hold due to inactivity timeout on manual control.");

    _target_angle[Servo::Pan ] = _actual_angle.at(Servo::Pan);
    _target_angle[Servo::Tilt] = _actual_angle.at(Servo::Tilt);

    _target_mode = Mode::PositionControl;

    return State::Hold;
  }

  return State::Teleop;
}

void Node::publish()
{
  if (_target_mode == Mode::PositionControl)
  {
    publish_mode_PositionControl(_pan_angle_mode_pub);
    publish_mode_PositionControl(_tilt_angle_mode_pub);
  }
  else
  {
    publish_mode_VelocityControl(_pan_angle_mode_pub);
    publish_mode_VelocityControl(_tilt_angle_mode_pub);
  }

  publish_AngularVelocity(_pan_angle_vel_pub, _target_angular_velocity.at(Servo::Pan));
  publish_AngularVelocity(_tilt_angle_vel_pub, _target_angular_velocity.at(Servo::Tilt));

  static auto const PAN_MIN_ANGLE = (get_parameter("pan_min_angle_deg").as_double() * deg).in(rad);
  static auto const PAN_MAX_ANGLE = (get_parameter("pan_max_angle_deg").as_double() * deg).in(rad);

  RCLCPP_DEBUG(get_logger(), "pan_rad = %0.2f, pan_max = %0.2f, pan_min = %0.2f", _target_angle.at(Servo::Pan).numerical_value_in(rad), PAN_MAX_ANGLE.numerical_value_in(rad), PAN_MIN_ANGLE.numerical_value_in(rad));

  if (_target_angle.at(Servo::Pan) > PAN_MAX_ANGLE)
    publish_Angle(_pan_angle_pub, PAN_MAX_ANGLE);
  else if (_target_angle.at(Servo::Pan) < PAN_MIN_ANGLE)
    publish_Angle(_pan_angle_pub, PAN_MIN_ANGLE);
  else
    publish_Angle(_pan_angle_pub, _target_angle.at(Servo::Pan));


  static auto const TILT_MIN_ANGLE = (get_parameter("tilt_min_angle_deg").as_double() * deg).in(rad);
  static auto const TILT_MAX_ANGLE = (get_parameter("tilt_max_angle_deg").as_double() * deg).in(rad);

  RCLCPP_DEBUG(get_logger(), "tilt_rad = %0.2f, tilt_max = %0.2f, tilt_min = %0.2f", _target_angle.at(Servo::Tilt).numerical_value_in(rad), TILT_MAX_ANGLE.numerical_value_in(rad), TILT_MIN_ANGLE.numerical_value_in(rad));

  if (_target_angle.at(Servo::Tilt) > TILT_MAX_ANGLE)
    publish_Angle(_tilt_angle_pub, TILT_MAX_ANGLE);
  else if (_target_angle.at(Servo::Tilt) < TILT_MIN_ANGLE)
    publish_Angle(_tilt_angle_pub, TILT_MIN_ANGLE);
  else
    publish_Angle(_tilt_angle_pub, _target_angle.at(Servo::Tilt));
}

void Node::publish_AngularVelocity(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr const pub, quantity<rad/s> const angular_velocity)
{
  std_msgs::msg::Float32 msg;
  msg.data = angular_velocity.numerical_value_in(rad/s);
  pub->publish(msg);
}

void Node::publish_Angle(rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr const pub, quantity<rad> const angle)
{
  std_msgs::msg::Float32 msg;
  msg.data = angle.numerical_value_in(rad);
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
