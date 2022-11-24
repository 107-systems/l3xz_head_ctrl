/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_head_ctrl/head/HeadController.h>

#include <l3xz_head_ctrl/head/state/Teleop.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head
{

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Controller::Controller()
: _head_state{new state::Teleop()}
{
  _head_state->onEnter();
}

Controller::~Controller()
{
  delete _head_state;
}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

ControllerOutput Controller::update(ControllerInput const & input, ControllerOutput const & prev_output)
{
  auto [next_head_state, next_output] = _head_state->update(input, prev_output);
    
  if (next_head_state != _head_state)
  {
    _head_state->onExit();

    delete _head_state;
    _head_state = next_head_state;
    
    _head_state->onEnter();
  }

  return next_output;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */