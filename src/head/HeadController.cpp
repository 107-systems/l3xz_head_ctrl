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

using namespace mx28ar;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

Controller::Controller(std::unique_ptr<MX28AR_Control> && mx28_ctrl,
                       rclcpp::Logger const logger,
                       dynamixelplusplus::Dynamixel::Id const pan_servo_id,
                       dynamixelplusplus::Dynamixel::Id const tilt_servo_id)
: _mx28_ctrl{std::move(mx28_ctrl)}
, _head_state{new state::Teleop(logger, pan_servo_id, tilt_servo_id)}
{
  _head_state->onEnter(*_mx28_ctrl);
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
  auto next_head_state = _head_state->update(*_mx28_ctrl, pan_angular_velocity, tilt_angular_velocity);
    
  if (next_head_state != _head_state)
  {
    _head_state->onExit(*_mx28_ctrl);

    delete _head_state;
    _head_state = next_head_state;
    
    _head_state->onEnter(*_mx28_ctrl);
  }
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head */