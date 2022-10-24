///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#include "GpsService.h"
#include <GpsService/GpsServiceMessages.hpp>
#include <CANFactory/CanMessage.h>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("gps_service_node");
  gps::GpsService service(node);
  while (true) {
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
