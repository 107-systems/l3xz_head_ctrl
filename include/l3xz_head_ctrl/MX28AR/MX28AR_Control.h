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

#include <map>
#include <memory>

#include <dynamixel++/dynamixel++.h>

#include "MX28AR_Const.h"

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::mx28ar
{

/**************************************************************************************
 * CLASS DECLARATION
 **************************************************************************************/

class MX28AR_Control : private dynamixelplusplus::SyncGroup
{
public:
  MX28AR_Control(dynamixelplusplus::SharedDynamixel dyn_ctrl,
                 dynamixelplusplus::Dynamixel::Id const pan_servo_id,
                 dynamixelplusplus::Dynamixel::Id const tilt_servo_id)
  : dynamixelplusplus::SyncGroup{dyn_ctrl, dynamixelplusplus::Dynamixel::IdVect{pan_servo_id, tilt_servo_id}}
  { }

  void setTorqueEnable (TorqueEnable const torque_enable);
  void setOperatingMode(OperatingMode const operating_mode);
  void setGoalPosition (float const pan_angle_deg, float const tilt_angle_deg);
  void setGoalVelocity (float const pan_velocity_rpm, float const tilt_velocity_rpm);

  std::tuple<float, float> getPresentPosition();
};

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::mx28ar */

#endif //L3XZ_HEAD_CTRL_MX28ARCONTROL_H
