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
  //TODO Support ISOName table
   sc::CANDevice device;
   device.Node = node;
   device.Address = node;
   device.Bus = 2;
   return device;
  //return Node.FindDevice(node);
}

void GpsServiceROS::Register()
{
  Subscribe<bridge::VehiclePosition>();
  Subscribe<bridge::DifferentialStatus>();
  Subscribe<bridge::RollYawRateVehicle>();
  Subscribe<bridge::RollYawRateImplement>();
  Subscribe<bridge::PitchAltitude>();
  Subscribe<bridge::DirectionSpeed>();
  Subscribe<bridge::BearingSpeed>();
  Subscribe<bridge::TimeDate>();
  Subscribe<bridge::TerrainCompensationVehicle>();
  Subscribe<bridge::TerrainCompensationImplement>();
  Subscribe<bridge::GpsStatusVehicle>();
  Subscribe<bridge::GpsStatusImplement>();
  Subscribe<bridge::SatellitesUsed>();
  Subscribe<bridge::ReceiverInfo>();
  Subscribe<bridge::HeadingMode>();
  Subscribe<bridge::MachineSelectedSpeed>();
  Subscribe<bridge::WheelBasedSpeedAndDistance>();
}
