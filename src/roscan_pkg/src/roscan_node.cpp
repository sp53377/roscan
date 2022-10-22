#include "CanBridge.hpp"
#include <GpsService/GpsServiceMessages.hpp>
#include <CANFactory/CanMessage.h>

void RegisterGpsService(CanBridge& bridge)
{
	bridge.Register<bridge::VehiclePosition>(2);
	bridge.Register1Cmd<bridge::DifferentialStatus>(2);
	bridge.Register1Cmd<bridge::RollYawRateVehicle>(2);
	bridge.Register1Cmd<bridge::RollYawRateImplement>(2);
	bridge.Register1Cmd<bridge::PitchAltitude>(2);
	bridge.Register<bridge::DirectionSpeed>(2);
	bridge.Register1Cmd<bridge::BearingSpeed>(2);
	bridge.Register<bridge::TimeDate>(2);
	bridge.Register1Cmd<bridge::TerrainCompensationVehicle>(2);
	bridge.Register1Cmd<bridge::TerrainCompensationImplement>(2);
	bridge.Register1Cmd<bridge::GpsStatusVehicle>(2);
	bridge.Register1Cmd<bridge::GpsStatusImplement>(2);
	bridge.Register1Cmd<bridge::SatellitesUsed>(2);
	bridge.Register1Cmd<bridge::ReceiverInfo>(2);
	bridge.Register1Cmd<bridge::HeadingMode>(2);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("roscan_node");
    CanBridge bridge(node);
    RegisterGpsService(bridge);
    while(true)
    {
      bridge.Step();
      rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
  return 0;
}