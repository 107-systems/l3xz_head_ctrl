/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_STATE_H_
#define HEAD_CONTROLLER_STATE_H_

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
  virtual ~StateBase() { }
  virtual void onEnter() { }
  virtual void onExit() { }
  virtual StateBase * update(float const pan_angular_velocity, float const tilt_angular_velocity) = 0;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head::state */

#endif /* HEAD_CONTROLLER_STATE_H_ */
