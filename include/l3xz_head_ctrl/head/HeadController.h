/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

#ifndef HEAD_CONTROLLER_H_
#define HEAD_CONTROLLER_H_

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include "state/StateBase.h"
#include "HeadControllerInput.h"
#include "HeadControllerOutput.h"

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
   Controller();
  ~Controller();

  ControllerOutput update(ControllerInput const & input, ControllerOutput const & prev_output);

private:
  state::StateBase * _head_state;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */

#endif /* HEAD_CONTROLLER_H_ */
