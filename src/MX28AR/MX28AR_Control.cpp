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

bool MX28AR_Control::setOperatingMode(Dynamixel::IdVect const & id_vect, OperatingMode const operating_mode)
{
  std::map<Dynamixel::Id, uint8_t> op_mode_data_map;

  for (auto id : id_vect)
    op_mode_data_map[id] = static_cast<uint8_t>(operating_mode);

  return (_dyn_ctrl->syncWrite(static_cast<uint16_t>(ControlTable::OperatingMode), op_mode_data_map) == Dynamixel::Error::None);
}

/**************************************************************************************
 * NAMESPACE
 **************************************************************************************/

} /* l3xz::mx28ar */
