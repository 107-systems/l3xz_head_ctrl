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

Teleop::Teleop(rclcpp::Logger const logger, dynamixelplusplus::Dynamixel::Id const pan_servo_id, dynamixelplusplus::Dynamixel::Id const tilt_servo_id)
: StateBase(logger, pan_servo_id, tilt_servo_id)
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

  std::map<dynamixelplusplus::Dynamixel::Id, float> const INITIAL_ID_RPM_MAP =
  {
    {_pan_servo_id,  0.0f},
    {_tilt_servo_id, 0.0f},
  };
  CHECK(!mx28_ctrl.setGoalVelocity(INITIAL_ID_RPM_MAP), "could not set initial pan/tilt servo velocity.");
}

void Teleop::onExit(MX28AR_Control & /* mx28_ctrl */)
{

}

StateBase * Teleop::update(MX28AR_Control & /* mx28_ctrl */, float const /* pan_angular_velocity */, float const /* tilt_angular_velocity */)
{
  /* TODO: Set desired velocity AND limit angles. */

  return this;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head::state */
