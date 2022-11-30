/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_H_
#define HEAD_CONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <memory>

#include <l3xz_head_ctrl/MX28AR/MX28AR_Control.h>

#include "state/StateBase.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class Controller
{
public:
   Controller(std::unique_ptr<mx28ar::MX28AR_Control> && mx28_ctrl);
  ~Controller();


  void update(float const pan_angular_velocity, float const tilt_angular_velocity);


private:
  std::unique_ptr<mx28ar::MX28AR_Control> _mx28_ctrl;
  state::StateBase * _head_state;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */

#endif /* HEAD_CONTROLLER_H_ */
