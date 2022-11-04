///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////

#pragma once
#include "GpsService.h"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <CANFactory/TwoBytePGNRequest.h>
#include <GpsService/GpsServiceBridges.hpp>

namespace gps
{
   class GpsServiceROS
   {   
   private:
      using SubscriptionBase_t = std::shared_ptr<rclcpp::SubscriptionBase>;
      using SubscriptionList = std::list<SubscriptionBase_t>;

      std::shared_ptr<rclcpp::Node> & Node;
      GpsService& Service;
      SubscriptionList Subscriptions;

   public:
      GpsServiceROS(std::shared_ptr<rclcpp::Node> & node, GpsService& service);

      virtual ~GpsServiceROS();

      void Register();

      sc::CANDevice GetDevice(sc::node_t node) const;

      template<typename T> typename rclcpp::Subscription<typename T::rostype>::SharedPtr Subscribe()
      {
	auto subscription = Node->create_subscription<typename T::rostype>(T::topic, 10, [&](const std::shared_ptr<typename T::rostype> msg){
		uint32_t frameId;
		int64_t timestamp;
		const auto& cppMsg = T::FromROS(*msg, &frameId, &timestamp);
		sc::node_t node = sc::GetSourceAddrFromFrameId(frameId);//TODO create a node index from a name table
		//std::cout << (int)node << " : "<< T::topic << std::endl;
		Service.Handle(node, cppMsg, timestamp);
	});
	Subscriptions.emplace_back(subscription);
	return subscription;
      }

      template<typename T>
      bool SendTwoBytePGNRequest(sc::addr_t address)
      {
	(void)address;
	return false;
      }
   };
}
