///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once
#include "GpsService.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <CANFactory/TwoBytePGNRequest.h>
#include <GpsService/GpsServiceMessages.hpp>

namespace gps
{
   class GpsServiceROS
   {   
   private:
      std::shared_ptr<rclcpp::Node> & Node;
      GpsService& Service;
      rclcpp::Subscription<typename bridge::VehiclePosition::rostype>::SharedPtr VehiclePositionSub;
      rclcpp::Subscription<typename bridge::DifferentialStatus::rostype>::SharedPtr DifferentialStatusSub;
      rclcpp::Subscription<typename bridge::RollYawRateVehicle::rostype>::SharedPtr RollYawRateVehicleSub;
      rclcpp::Subscription<typename bridge::RollYawRateImplement::rostype>::SharedPtr RollYawRateImplementSub;
      rclcpp::Subscription<typename bridge::PitchAltitude::rostype>::SharedPtr PitchAltitudeSub;
      rclcpp::Subscription<typename bridge::DirectionSpeed::rostype>::SharedPtr DirectionSpeedSub;
      rclcpp::Subscription<typename bridge::BearingSpeed::rostype>::SharedPtr BearingSpeedSub;
      rclcpp::Subscription<typename bridge::TimeDate::rostype>::SharedPtr TimeDateSub;
      rclcpp::Subscription<typename bridge::TerrainCompensationVehicle::rostype>::SharedPtr TerrainCompensationVehicleSub;
      rclcpp::Subscription<typename bridge::TerrainCompensationImplement::rostype>::SharedPtr TerrainCompensationImplementSub;
      rclcpp::Subscription<typename bridge::GpsStatusVehicle::rostype>::SharedPtr GpsStatusVehicleSub;
      rclcpp::Subscription<typename bridge::GpsStatusImplement::rostype>::SharedPtr GpsStatusImplementSub;
      rclcpp::Subscription<typename bridge::SatellitesUsed::rostype>::SharedPtr SatellitesUsedSub;
      rclcpp::Subscription<typename bridge::ReceiverInfo::rostype>::SharedPtr ReceiverInfoSub;
      rclcpp::Subscription<typename bridge::HeadingMode::rostype>::SharedPtr HeadingModeSub;
      rclcpp::Subscription<typename bridge::MachineSelectedSpeed::rostype>::SharedPtr MachineSelectedSpeedSub;
      rclcpp::Subscription<typename bridge::WheelBasedSpeedAndDistance::rostype>::SharedPtr WheelBasedSpeedAndDistanceSub;

   public:
      GpsServiceROS(std::shared_ptr<rclcpp::Node> & node, GpsService& service);

      virtual ~GpsServiceROS();

      void Register();

      sc::CANDevice GetDevice(sc::node_t node) const;

      template<typename T> typename rclcpp::Subscription<typename T::rostype>::SharedPtr Subscribe() const
      {
	return Node->create_subscription<typename T::rostype>(T::topic, 10, [&](const std::shared_ptr<typename T::rostype> msg){
		uint8_t source;
		int64_t timestamp;
		const auto& cppMsg = T::FromROS(*msg, &source, &timestamp);
		sc::node_t node = source;
		Service.Handle(node, cppMsg, timestamp);
	});
      }

      template<typename T>
      bool SendTwoBytePGNRequest(sc::addr_t address)
      {
	(void)address;
	return false;
      }
   };
}
