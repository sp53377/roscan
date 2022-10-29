///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#pragma once

#include "can_interfaces/srv/register_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include <CANFactory/CanMessage.h>
#include <CANFactory/CANTypes.h>
#include <boost/interprocess/ipc/message_queue.hpp>
#include "std_msgs/msg/string.hpp"
#include <map>
#include <functional>

class CanBridge
{
private:
  static constexpr uint32_t PGN_MASK = 0x00FFFF00;
  static constexpr int DEFAULT_QUEUE_SIZE = 10;//Number of messages to queue for publisher

  using ForwarderFn = std::function<void(const sc::CanMessage_t&)>;
  
  using Bridge_t = std::shared_ptr<rclcpp::SubscriptionBase>;
  using BridgeList = std::list<Bridge_t>;

  struct Subscription_t
  {
    uint32_t Handle = INVALID_HANDLE;
    uint8_t Channel = 0;
    uint32_t FrameId = 0;
    uint32_t FrameIdMask = 0;
    uint64_t Mask = 0;
    uint64_t Match = 0;
    ForwarderFn Fn;
  };

  using SubscriptionMultimap = std::multimap<uint32_t, Subscription_t>;
  using SubscriptionPair = std::pair<uint32_t, Subscription_t>;
  using SubscriptionIterator = SubscriptionMultimap::iterator;
  using SubscriptionRange = std::pair<SubscriptionIterator, SubscriptionIterator>;

  std::shared_ptr<rclcpp::Node> & Node;
  std::shared_ptr<boost::interprocess::message_queue> RxQueue;
  std::shared_ptr<boost::interprocess::message_queue> TxQueue;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Publish;
  rclcpp::Client<can_interfaces::srv::RegisterMsg>::SharedPtr RegisterMsgClient;
  rclcpp::TimerBase::SharedPtr Timer;
  SubscriptionMultimap Subscriptions;
  BridgeList Bridges;
  size_t Count = 0;
  uint8_t SourceAddress = 0x99;//TODO Do Address Claim

  static std::shared_ptr<boost::interprocess::message_queue> MakeRxQueue();
  static std::shared_ptr<boost::interprocess::message_queue> MakeTxQueue();

  void WaitForConnect();
  void WaitForService();
  void OnTimeout();
  void HandleMessage(const sc::CanMessage_t & msg);

  template<typename T> ForwarderFn MakeForwarder()
  {
	auto publisher = Node->create_publisher<typename T::rostype>(T::topic, 10);//TODO What QoS do we need?
	return [publisher](const sc::CanMessage_t& msg){
		auto rosMsg = T::ToROS(T::cantype::FromCAN(sc::ToCANMsgType(msg)));
		rosMsg.can_data.frame_id = msg.FrameId;
		rosMsg.can_data.timestamp = msg.Timestamp;
		publisher->publish(rosMsg);
	};
  }

  ForwarderFn MakeGenericForwarder(const char* topic);

  static bool Matches(const Subscription_t & subscription, const sc::CanMessage_t & msg);

  void Send(const sc::CanMessage_t& out);

  template <typename T> void Send(const T& in, uint8_t bus, uint8_t destinationAddress, bool global)
  {
	sc::CanMessage_t out;
	out.Length = static_cast<uint8_t>(sizeof(T));
	assert(out.Length<=8);
	if(out.Length<=8)
	{
		memcpy(out.Bytes,&in,out.Length);
		out.Channel = bus;
		out.FrameId = global ? sc::MakeFrameId(T::PGN, T::PRIORITY, SourceAddress) : sc::MakeFrameId(T::PGN,T::PRIORITY,SourceAddress, destinationAddress);
		out.Timestamp = 0;
		Send(out);
	}
  }
public:
  static constexpr uint32_t INVALID_HANDLE = 0;

  explicit CanBridge(std::shared_ptr<rclcpp::Node> & node);

  Subscription_t Register(uint8_t channel, int32_t pgn, uint64_t mask, uint64_t match, uint8_t address, const ForwarderFn& fn = ForwarderFn());
  Subscription_t Register(uint8_t channel, int32_t pgn, uint8_t address, const ForwarderFn& fn = ForwarderFn());
  Subscription_t Register1Cmd(uint8_t channel, int32_t pgn, uint8_t cmdByte, uint8_t address, const ForwarderFn& fn = ForwarderFn());
  Subscription_t Register2Cmd(uint8_t channel, int32_t pgn, uint8_t cmdByte1, uint8_t cmdByte2, uint8_t address, const ForwarderFn& fn = ForwarderFn());
  template<typename T> Subscription_t Register(uint8_t channel, uint8_t address = sc::NULL_ADDRESS)
  {
    return Register(channel, T::cantype::PGN, address, MakeForwarder<T>());
  }

  template<typename T> Subscription_t Register1Cmd(uint8_t channel, uint8_t address = sc::NULL_ADDRESS)
  {
    return Register1Cmd(channel, T::cantype::PGN, T::cantype::CMD1, address, MakeForwarder<T>());
  }

  template<typename T> Subscription_t Register2Cmd(uint8_t channel, uint8_t address = sc::NULL_ADDRESS)
  {
    return Register2Cmd(channel, T::cantype::PGN, T::cantype::CMD1, T::cantype::CMD2, address, MakeForwarder<T>());
  }

  //Generic message registration
  Subscription_t GenericRegister(uint8_t channel, int32_t pgn, uint64_t mask, uint64_t match, uint8_t address, const char* topic);
  Subscription_t GenericRegister(uint8_t channel, int32_t pgn, uint8_t address, const char* topic);
  Subscription_t GenericRegister1Cmd(uint8_t channel, int32_t pgn, uint8_t cmdByte, uint8_t address, const char* topic);
  Subscription_t GenericRegister2Cmd(uint8_t channel, int32_t pgn, uint8_t cmdByte1, uint8_t cmdByte2, uint8_t address, const char* topic);


  template<typename T> std::shared_ptr<rclcpp::SubscriptionBase> BridgeGlobal(uint8_t bus)
  {
	auto bridge = Node->create_subscription<typename T::rostype>(T::topic, 10, [&,bus](const std::shared_ptr<typename T::rostype> msg){
		auto cppMsg = T::FromROS(*msg, nullptr, nullptr);
		auto canMsg = T::cantype::ToCAN(cppMsg);
		Send(canMsg, bus, 0xFF, true);
	});
	Bridges.emplace_back(bridge);
	return bridge;
  }

  template<typename T> std::shared_ptr<rclcpp::SubscriptionBase> Bridge(uint8_t bus)
  {
	auto bridge = Node->create_subscription<typename T::rostype>(T::topic, 10, [&,bus](const std::shared_ptr<typename T::rostype> msg){
		auto cppMsg = T::FromROS(*msg, nullptr, nullptr);
		auto canMsg = T::cantype::ToCAN(cppMsg);
		Send(canMsg, bus, msg->destination_addr, false);
	});
	Bridges.emplace_back(bridge);
	return bridge;
  }

  void Step();
};
