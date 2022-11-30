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
  virtual ~Teleop() { }
  virtual void onEnter() override;
  virtual void onExit() override;
  virtual StateBase * update(mx28ar::MX28AR_Control & mx28_ctrl, float const pan_angular_velocity, float const tilt_angular_velocity) override;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head::state */

#endif /* HEAD_TELEOP_STATE_H_ */
