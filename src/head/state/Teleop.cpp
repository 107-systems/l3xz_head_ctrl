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

using namespace mx28ar;

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

void Teleop::onEnter(MX28AR_Control & /* mx28_ctrl */)
{

}

void Teleop::onExit(MX28AR_Control & /* mx28_ctrl */)
{

}

StateBase * Teleop::update(MX28AR_Control & /* mx28_ctrl */, float const /* pan_angular_velocity */, float const /* tilt_angular_velocity */)
{
  return this;
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::head::state */
