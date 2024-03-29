/**
 * Copyright (c) 2022 LXRobotics GmbH.
 * Author: Alexander Entinger <alexander.entinger@lxrobotics.com>
 * Contributors: https://github.com/107-systems/l3xz_head_ctrl/graphs/contributors.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <rclcpp/rclcpp.hpp>

#include <l3xz_head_ctrl/Node.h>

/**************************************************************************************
 * MAIN
 **************************************************************************************/

int main(int argc, char * argv[]) try
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<l3xz::head::Node>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return EXIT_SUCCESS;
}
catch (std::runtime_error const & err)
{
  std::cerr << "Exception (std::runtime_error) caught: " << err.what() << std::endl;
  std::cerr << "Terminating ..." << std::endl;
  return EXIT_FAILURE;
}
catch (...)
{
  std::cerr << "Unhandled exception caught." << std::endl;
  std::cerr << "Terminating ..." << std::endl;
  return EXIT_FAILURE;
}
