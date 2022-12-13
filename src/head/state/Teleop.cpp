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
  mx28_ctrl.setTorqueEnable (TorqueEnable::Off);
  mx28_ctrl.setOperatingMode(OperatingMode::VelocityControlMode);
  mx28_ctrl.setTorqueEnable (TorqueEnable::On);
  mx28_ctrl.setGoalVelocity (0.0, 0.0);
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
  auto [pan_angle_deg, tilt_angle_deg] = mx28_ctrl.getPresentPosition();

  if ((pan_angle_deg < _pan_min_angle_deg) && (pan_angular_velocity_dps < 0.0f))
    _goal_velocity_rpm[_pan_servo_id] = 0.0f;
  if ((pan_angle_deg > _pan_max_angle_deg) && (pan_angular_velocity_dps > 0.0f))
    _goal_velocity_rpm[_pan_servo_id] = 0.0f;
  if ((tilt_angle_deg < _tilt_min_angle_deg) && (tilt_angular_velocity_dps < 0.0f))
    _goal_velocity_rpm[_tilt_servo_id] = 0.0f;
  if ((tilt_angle_deg > _tilt_max_angle_deg) && (tilt_angular_velocity_dps > 0.0f))
    _goal_velocity_rpm[_tilt_servo_id] = 0.0f;

  /* Write the computed RPM value to the Dynamixel MX-28AR
   * servos of the pan/tilt head.
   */
  mx28_ctrl.setGoalVelocity(_goal_velocity_rpm[_pan_servo_id], _goal_velocity_rpm[_tilt_servo_id]);

  return this;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head::state */
