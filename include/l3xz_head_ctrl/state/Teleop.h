/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

#ifndef HEAD_TELEOP_STATE_H_
#define HEAD_TELEOP_STATE_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "StateBase.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head::state
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Teleop : public StateBase
{
public:
  Teleop(rclcpp::Logger const logger, dynamixelplusplus::Dynamixel::Id const pan_servo_id, dynamixelplusplus::Dynamixel::Id const tilt_servo_id);
  virtual ~Teleop() { }


  virtual void onEnter(mx28ar::MX28AR_Control & mx28_ctrl) override;
  virtual void onExit(mx28ar::MX28AR_Control & mx28_ctrl) override;
  virtual StateBase * update(mx28ar::MX28AR_Control & mx28_ctrl, float const pan_angular_velocity_dps, float const tilt_angular_velocity_dps) override;

private:
  std::map<dynamixelplusplus::Dynamixel::Id, float> _goal_velocity_rpm;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head::state */

#endif /* HEAD_TELEOP_STATE_H_ */
