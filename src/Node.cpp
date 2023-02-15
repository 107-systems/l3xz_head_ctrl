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
, _state{State::Teleop}
, _prev_ctrl_loop_timepoint{std::chrono::steady_clock::now()}
{
  /* Configure subscribers and publishers. */

  _head_sub = create_subscription<geometry_msgs::msg::Twist>
    ("/l3xz/cmd_vel_head", 1,
    [this](geometry_msgs::msg::Twist::SharedPtr const msg)
    {
      _pan_angular_velocity_rad_per_sec  = msg->angular.z;
      _tilt_angular_velocity_rad_per_sec = msg->angular.y;
    });

  _head_io_pub = create_publisher<l3xz_ros_dynamixel_bridge::msg::HeadVelocity>("/l3xz/head/velocity/target", 1);

  /* Configure periodic control loop function. */

  _ctrl_loop_timer = create_wall_timer
    (std::chrono::milliseconds(CTRL_LOOP_RATE.count()),
     [this]()
     {
       this->ctrl_loop();
     });

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


  l3xz_ros_dynamixel_bridge::msg::HeadVelocity head_vel_msg;

  switch(_state)
  {
    case State::Teleop:
    {
      head_vel_msg.pan_vel_rad_per_sec = _pan_angular_velocity_rad_per_sec;
      head_vel_msg.tilt_vel_rad_per_sec = _tilt_angular_velocity_rad_per_sec;
    }
    break;
    default:
    {
      head_vel_msg.pan_vel_rad_per_sec = 0.0f;
      head_vel_msg.tilt_vel_rad_per_sec = 0.0f;
    }
    break;
  }

  _head_io_pub->publish(head_vel_msg);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */