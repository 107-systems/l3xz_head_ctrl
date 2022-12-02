/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_head_ctrl/state/Teleop.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head::state
{

using namespace dynamixelplusplus;
using namespace mx28ar;

/**************************************************************************************
 * DEFINE
 **************************************************************************************/

#define CHECK(cond,err_msg)           \
  if (cond) {                         \
    RCLCPP_ERROR(_logger, (err_msg)); \
    rclcpp::shutdown();               \
  }

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Teleop::Teleop(rclcpp::Logger const logger,
               dynamixelplusplus::Dynamixel::Id const pan_servo_id,
               dynamixelplusplus::Dynamixel::Id const tilt_servo_id,
               float const pan_min_angle_deg,
               float const pan_max_angle_deg,
               float const tilt_min_angle_deg,
               float const tilt_max_angle_deg)
: StateBase(logger, pan_servo_id, tilt_servo_id, pan_min_angle_deg, pan_max_angle_deg, tilt_min_angle_deg, tilt_max_angle_deg)
, _goal_velocity_rpm
{
  {_pan_servo_id,  0.0f},
  {_tilt_servo_id, 0.0f},
}
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Teleop::onEnter(MX28AR_Control & mx28_ctrl)
{
  CHECK(!mx28_ctrl.setTorqueEnable(_pan_tilt_id_vect, TorqueEnable::Off), "could not disable torque for pan/tilt servos.");
  CHECK(!mx28_ctrl.setOperatingMode(_pan_tilt_id_vect, OperatingMode::VelocityControlMode), "could not configure pan/tilt servos for velocity control mode.");
  CHECK(!mx28_ctrl.setTorqueEnable(_pan_tilt_id_vect, TorqueEnable::On), "could not enable torque for pan/tilt servos.");
  CHECK(!mx28_ctrl.setGoalVelocity(_goal_velocity_rpm), "could not set initial pan/tilt servo velocity.");
}

void Teleop::onExit(MX28AR_Control & /* mx28_ctrl */)
{

}

StateBase * Teleop::update(MX28AR_Control & mx28_ctrl, float const pan_angular_velocity_rad_per_sec, float const tilt_angular_velocity_rad_per_sec)
{
  float const pan_angular_velocity_dps  = pan_angular_velocity_rad_per_sec  * 180.0f / M_PI;
  float const tilt_angular_velocity_dps = tilt_angular_velocity_rad_per_sec * 180.0f / M_PI;

  static float constexpr DPS_per_RPM = 360.0f / 60.0f;
  _goal_velocity_rpm[_pan_servo_id]  = pan_angular_velocity_dps / DPS_per_RPM;
  _goal_velocity_rpm[_tilt_servo_id] = tilt_angular_velocity_dps / DPS_per_RPM;

  /* Checking current head position and stopping if either
   * pan or tilt angle would exceed the maximum allowed angle.
   */
  std::map<Dynamixel::Id, float> actual_head_position_deg;
  CHECK(!mx28_ctrl.getPresentPosition(_pan_tilt_id_vect, actual_head_position_deg), "could not read current position for pan/tilt servo.");
  CHECK(!actual_head_position_deg.count(_pan_servo_id), "could no position data for pan servo.");
  CHECK(!actual_head_position_deg.count(_tilt_servo_id), "could no position data for tilt servo.");

  if ((actual_head_position_deg.at(_pan_servo_id) < _pan_min_angle_deg) && (pan_angular_velocity_dps < 0.0f))
    _goal_velocity_rpm[_pan_servo_id] = 0.0f;
  if ((actual_head_position_deg.at(_pan_servo_id) > _pan_max_angle_deg) && (pan_angular_velocity_dps > 0.0f))
    _goal_velocity_rpm[_pan_servo_id] = 0.0f;
  if ((actual_head_position_deg.at(_tilt_servo_id) < _tilt_min_angle_deg) && (tilt_angular_velocity_dps < 0.0f))
    _goal_velocity_rpm[_tilt_servo_id] = 0.0f;
  if ((actual_head_position_deg.at(_tilt_servo_id) > _tilt_max_angle_deg) && (tilt_angular_velocity_dps > 0.0f))
    _goal_velocity_rpm[_tilt_servo_id] = 0.0f;

  /* Write the computed RPM value to the Dynamixel MX-28AR
   * servos of the pan/tilt head.
   */
  CHECK(!mx28_ctrl.setGoalVelocity(_goal_velocity_rpm), "could not set pan/tilt servo velocity.");

  return this;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head::state */
