/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <l3xz_head_ctrl/MX28AR/MX28AR_Control.h>

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

namespace l3xz::mx28ar
{

using namespace dynamixelplusplus;

/**************************************************************************************
 * CTOR/DTOR
 **************************************************************************************/

MX28AR_Control::MX28AR_Control(std::unique_ptr<Dynamixel> && dyn_ctrl)
: _dyn_ctrl(std::move(dyn_ctrl))
{

}

/**************************************************************************************
 * PUBLIC MEMBER FUNCTIONS
 **************************************************************************************/

bool MX28AR_Control::setTorqueEnable(Dynamixel::IdVect const & id_vect, TorqueEnable const torque_enable)
{
  std::map<Dynamixel::Id, uint8_t> torque_enable_data_map;

  for (auto id : id_vect)
    torque_enable_data_map[id] = static_cast<uint8_t>(torque_enable);

  return (_dyn_ctrl->syncWrite(static_cast<uint16_t>(ControlTable::TorqueEnable), torque_enable_data_map) == Dynamixel::Error::None);
}

bool MX28AR_Control::setOperatingMode(Dynamixel::IdVect const & id_vect, OperatingMode const operating_mode)
{
  std::map<Dynamixel::Id, uint8_t> op_mode_data_map;

  for (auto id : id_vect)
    op_mode_data_map[id] = static_cast<uint8_t>(operating_mode);

  return (_dyn_ctrl->syncWrite(static_cast<uint16_t>(ControlTable::OperatingMode), op_mode_data_map) == Dynamixel::Error::None);
}

bool MX28AR_Control::setGoalPosition(std::map<Dynamixel::Id, float> const & id_angle_map)
{
  std::map<Dynamixel::Id, uint32_t> goal_position_data_map;

  for (auto [id, angle_deg] : id_angle_map)
  {
    auto isValidAngle = [](float const a) { return (a >= 0.0f && a <= 360.0f); };
    if (!isValidAngle(angle_deg))
      return false;
    goal_position_data_map[id] = static_cast<uint32_t>((angle_deg * 4096.0f) / 360.0f);
  }

  return (_dyn_ctrl->syncWrite(static_cast<uint16_t>(ControlTable::GoalPosition), goal_position_data_map) == Dynamixel::Error::None);
}

bool MX28AR_Control::getPresentPosition(Dynamixel::IdVect const & id_vect, std::map<Dynamixel::Id, float> & id_angle_map)
{
  std::map<Dynamixel::Id, uint32_t> id_angle_raw_map;
  if (_dyn_ctrl->syncRead(static_cast<uint16_t>(ControlTable::PresentPosition), id_vect, id_angle_raw_map) != Dynamixel::Error::None)
    return false;

  for (auto [id, angle_raw] : id_angle_raw_map)
    id_angle_map[id] = static_cast<float>(angle_raw) * 360.0f / 4096.0f;

  return true;
}

bool MX28AR_Control::setGoalVelocity(std::map<dynamixelplusplus::Dynamixel::Id, float> const & id_rpm_map)
{
  std::map<Dynamixel::Id, uint32_t> goal_velocity_data_map;

  static float const RPM_per_LSB = 0.229f;
  static float const MAX_VELOCITY_rpm = RPM_per_LSB * 1023.0f;
  static float const MIN_VELOCITY_rpm = RPM_per_LSB * 1023.0f * (-1.0);

  auto limit_velocity = [](float const rpm)
  {
         if (rpm < MIN_VELOCITY_rpm) return MIN_VELOCITY_rpm;
    else if (rpm > MAX_VELOCITY_rpm) return MAX_VELOCITY_rpm;
    else                             return rpm;
  };

  for (auto [id, rpm] : id_rpm_map)
  {
    auto const rpm_limited = limit_velocity(rpm);
    int32_t const rpm_lsb_signed = static_cast<int32_t>(rpm_limited * RPM_per_LSB);
    goal_velocity_data_map[id] = static_cast<uint32_t>(rpm_lsb_signed);
  }

  return (_dyn_ctrl->syncWrite(static_cast<uint16_t>(ControlTable::GoalVelocity), goal_velocity_data_map) == Dynamixel::Error::None);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::mx28ar */
