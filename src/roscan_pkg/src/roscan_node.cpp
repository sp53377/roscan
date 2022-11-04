///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#include "CanBridge.hpp"
#include <GpsService/GpsServiceBridges.hpp>
#include <CANFactory/CanMessageBridges.hpp>
#include <CANFactory/CanMessage.h>

void RegisterGpsService(CanBridge & bridge)
{
	//Allow generic message sends on CAN1 & 2
  bridge.AddGenericBridge(CanBridge::CAN1|CanBridge::CAN2);
	//Register for a generic PGN request message (example)
  bridge.GenericRegister(2, 0xEA00, sc::NULL_ADDRESS, "/can/PGNReq");
	//Allow PGN Request sends to the global address
  bridge.BridgeGlobal<bridge::GlobalPgnRequest>(2);
	//Register for "typed" incomming PGN Requests on the PgnRequestIn topic
  bridge.Register<bridge::PgnRequestIn>(2);
	//Allow "typed" PGN Request sends to the destination address
  bridge.Bridge<bridge::PgnRequest>(2);
	//Register for "typed" incomming PGN Requests on the PgnRequest topic
  bridge.Register<bridge::PgnRequest>(2);

	//Bridge GPS "typed" messages from CAN to ROS
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
  
  //The roscan_node is dependency injected into the bridge
  auto node = rclcpp::Node::make_shared("roscan_node");
  CanBridge bridge(node);

  //Do the global registrations
  //TODO can this be a plugin with shared libraries somehow?
  RegisterGpsService(bridge);

  while (true) {
	//Process incomming messages from queues
    bridge.Step();
	//Process any ros node events
    rclcpp::spin_some(node);
  }
  rclcpp::shutdown();
  return 0;
}
