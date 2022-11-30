/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDES
 **************************************************************************************/

#include <l3xz_head_ctrl/head/state/Teleop.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::head::state
{

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Teleop::onEnter()
{

}

void Teleop::onExit()
{

}

StateBase * Teleop::update(mx28ar::MX28AR_Control & /* mx28_ctrl */, float const /* pan_angular_velocity */, float const /* tilt_angular_velocity */)
{
  return this;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head::state */
