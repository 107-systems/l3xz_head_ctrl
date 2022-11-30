/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
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

void Controller::update(float const pan_angular_velocity, float const tilt_angular_velocity)
{
  auto next_head_state = _head_state->update(pan_angular_velocity, tilt_angular_velocity);
    
  if (next_head_state != _head_state)
  {
    _head_state->onExit();

    delete _head_state;
    _head_state = next_head_state;
    
    _head_state->onEnter();
  }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */