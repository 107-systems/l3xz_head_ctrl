/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_STATE_H_
#define HEAD_CONTROLLER_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <dynamixel++/Dynamixel++.h>

#include <l3xz_head_ctrl/MX28AR/MX28AR_Control.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class StateBase
{
public:
  StateBase(rclcpp::Logger const logger, dynamixelplusplus::Dynamixel::Id const pan_servo_id, dynamixelplusplus::Dynamixel::Id const tilt_servo_id)
  : _logger{logger}
  , _pan_servo_id{pan_servo_id}
  , _tilt_servo_id{tilt_servo_id}
  , _pan_tilt_id_vect{_pan_servo_id, _tilt_servo_id}
  { }
  virtual ~StateBase() { }


  virtual void onEnter(mx28ar::MX28AR_Control & mx28_ctrl) = 0;
  virtual void onExit(mx28ar::MX28AR_Control & mx28_ctrl) = 0;
  virtual StateBase * update(mx28ar::MX28AR_Control & mx28_ctrl, float const pan_angular_velocity_dps, float const tilt_angular_velocity_dps) = 0;


protected:
  rclcpp::Logger const _logger;
  dynamixelplusplus::Dynamixel::Id const _pan_servo_id;
  dynamixelplusplus::Dynamixel::Id const _tilt_servo_id;
  dynamixelplusplus::Dynamixel::IdVect const _pan_tilt_id_vect;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head::state */

#endif /* HEAD_CONTROLLER_STATE_H_ */
