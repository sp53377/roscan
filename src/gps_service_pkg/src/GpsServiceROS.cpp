///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#include "GpsServiceROS.h"


using namespace gps;

GpsServiceROS::GpsServiceROS(std::shared_ptr<rclcpp::Node> & node, GpsService & service)
: Node(node),
  Service(service)
{
  Register();
}

GpsServiceROS::~GpsServiceROS()
{
}

sc::CANDevice GpsServiceROS::GetDevice(sc::node_t node) const
{
	(void)node;
  return sc::CANDevice{};
  //TODO Support ISOName table
  //return Node.FindDevice(node);
}

void GpsServiceROS::Register()
{
  VehiclePositionSub = Subscribe<bridge::VehiclePosition>();
  DifferentialStatusSub = Subscribe<bridge::DifferentialStatus>();
  RollYawRateVehicleSub = Subscribe<bridge::RollYawRateVehicle>();
  RollYawRateImplementSub = Subscribe<bridge::RollYawRateImplement>();
  PitchAltitudeSub = Subscribe<bridge::PitchAltitude>();
  DirectionSpeedSub = Subscribe<bridge::DirectionSpeed>();
  BearingSpeedSub = Subscribe<bridge::BearingSpeed>();
  TimeDateSub = Subscribe<bridge::TimeDate>();
  TerrainCompensationVehicleSub = Subscribe<bridge::TerrainCompensationVehicle>();
  TerrainCompensationImplementSub = Subscribe<bridge::TerrainCompensationImplement>();
  GpsStatusVehicleSub = Subscribe<bridge::GpsStatusVehicle>();
  GpsStatusImplementSub = Subscribe<bridge::GpsStatusImplement>();
  SatellitesUsedSub = Subscribe<bridge::SatellitesUsed>();
  ReceiverInfoSub = Subscribe<bridge::ReceiverInfo>();
  HeadingModeSub = Subscribe<bridge::HeadingMode>();
  MachineSelectedSpeedSub = Subscribe<bridge::MachineSelectedSpeed>();
  WheelBasedSpeedAndDistanceSub = Subscribe<bridge::WheelBasedSpeedAndDistance>();
}
