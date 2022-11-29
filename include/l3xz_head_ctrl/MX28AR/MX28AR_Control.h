/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

#ifndef L3XZ_HEAD_CTRL_MX28ARCONTROL_H
#define L3XZ_HEAD_CTRL_MX28ARCONTROL_H

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <memory>

#include <dynamixel++/Dynamixel++.h>

#include "MX28AR_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::mx28ar
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MX28AR_Control
{
public:
  MX28AR_Control(std::unique_ptr<dynamixelplusplus::Dynamixel> && dyn_ctrl);

  bool setOperatingMode(dynamixelplusplus::Dynamixel::IdVect const & id_vect, OperatingMode const operating_mode);

private:
  std::unique_ptr<dynamixelplusplus::Dynamixel> _dyn_ctrl;
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::mx28ar */

#endif //L3XZ_HEAD_CTRL_MX28ARCONTROL_H
