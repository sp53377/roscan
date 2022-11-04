///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once

#include "GpsModel.h"
#include <CANFactory/CanMessageBridges.hpp>
#include <CANFactory/GenericFactory.hpp>
#include <GpsService/GpsServiceBridges.hpp>
#include "rclcpp/rclcpp.hpp"
#include <CommonCore/CommonTypes.h>
#include <map>

namespace gps
{
   class GpsServiceROS;
   class GpsService
   {
   public:
      explicit GpsService(std::shared_ptr<rclcpp::Node> & node);
      virtual ~GpsService();

      GpsService(const GpsService&) = delete;
      GpsService& operator=(const GpsService&) = delete;

      void GpsDevicesReq(const hash_t& responseTopic);

      GpsModel& FindModel(sc::node_t sourceNode);

      template<typename T>
      void Handle(sc::node_t sourceNode, const T& event, const int64_t& timestamp)
      {
         FindModel(sourceNode).Handle(event, timestamp);
      }

      void HandleEpochReceived(const int64_t& timestamp);

      //This is for handling non-GPS messages in all GPS models
      template<typename T>
      void HandleGlobal(const T& event, const int64_t& timestamp)
      {
         for(auto& pair:Devices)
         {
            pair.second.Handle(event, timestamp);
         }
      }

      void PublishGpsEpoch(const gps::GpsEpoch& epoch);
      void PublishGpsEx(const gps::GpsEx& ex);
      void PublishGpsDevice(const gps::GpsDevice& device);

   private:
      typedef std::map<sc::node_t, GpsModel> ModelMap;
      std::shared_ptr<rclcpp::Node> & Node;
      GpsServiceROS* CAN;
      ModelMap Devices;
      GpsModel NullModel;
      int64_t LastUpdate = 0;
      rclcpp::Publisher<bridge::GpsEpoch::rostype>::SharedPtr EpochPublisher;
      rclcpp::Publisher<bridge::GpsEx::rostype>::SharedPtr ExPublisher;
      rclcpp::Publisher<bridge::GpsDevice::rostype>::SharedPtr DevicePublisher;
      rclcpp::Publisher<bridge::GlobalPgnRequest::rostype>::SharedPtr PgnReqPublisher;
      rclcpp::Publisher<can_interfaces::msg::GenericMsg>::SharedPtr GenericPublisher;
   };
}
