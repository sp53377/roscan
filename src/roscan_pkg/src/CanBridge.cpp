///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#include "CanBridge.hpp"
#include <can_interfaces/msg/generic_msg.hpp>
#include <CANFactory/CANTypes.h>
#include <CANFactory/CanQueues.hpp>
#include <chrono>
using namespace std::chrono_literals;
using namespace boost::interprocess;

CanBridge::CanBridge(std::shared_ptr<rclcpp::Node> & node)
: Node(node),
  Publish(Node->create_publisher<std_msgs::msg::String>("topic", 10)),
  RegisterMsgClient(Node->create_client<can_interfaces::srv::RegisterMsg>("register_msg")),
  Timer(Node->create_wall_timer(500ms, std::bind(&CanBridge::OnTimeout, this)))
{
  WaitForService();
  WaitForConnect();
}

std::shared_ptr<message_queue> CanBridge::MakeRxQueue()
{
  std::shared_ptr<message_queue> queue;
  try {
    queue = std::make_shared<message_queue>(open_only, sc::CanRx);
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "rclcpp"), "Couldn't connect to canbus rx queue retrying...");
      std::cerr << e.what() << '\n';
  }
  return queue;
}

std::shared_ptr<boost::interprocess::message_queue> CanBridge::MakeTxQueue()
{
  std::shared_ptr<message_queue> queue;
  do {
    try {
      queue = std::make_shared<message_queue>(open_only, sc::CanTx);
    } catch (const std::exception & e) {
      std::cout << "Couldn't connect to canbus tx queue retrying..." << std::endl;
      std::cerr << e.what() << '\n';
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  } while (!queue);
  return queue;
}

void CanBridge::WaitForConnect()
{
  RxQueue = MakeRxQueue();
  TxQueue = MakeTxQueue();
}

void CanBridge::WaitForService()
{
  while (!RegisterMsgClient->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        rclcpp::get_logger(
          "rclcpp"), "Interrupted while waiting for the register_msg service. Exiting.");
    }
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
  }
}

void CanBridge::OnTimeout()
{
  auto message = std_msgs::msg::String();
  message.data = "Got Message " + std::to_string(Count);
  RCLCPP_INFO(Node->get_logger(), "Publishing: '%s'", message.data.c_str());
  Publish->publish(message);
}

void CanBridge::Step()
{
  sc::CanMessage_t msg;
  std::size_t receivedSize;
  unsigned int priority = 0;
  if (RxQueue->try_receive(&msg, sizeof(sc::CanMessage_t), receivedSize, priority)) {
    if (receivedSize == sizeof(sc::CanMessage_t)) {
      HandleMessage(msg);
      Count++;
    }
  }
}

CanBridge::Subscription_t CanBridge::Register(
  uint8_t channel, int32_t pgn, uint64_t mask,
  uint64_t match, uint8_t address, const ForwarderFn& fn)
{
  Subscription_t subscription;
  subscription.Channel = channel;
  subscription.FrameId = address == sc::NULL_ADDRESS ? (pgn << 8) : ((pgn << 8) | address);
  subscription.FrameIdMask = address == sc::NULL_ADDRESS ? 0xFFFF00 : 0xFFFFFF;
  subscription.Mask = mask;
  subscription.Match = match;
  subscription.Fn = fn;

  auto request = std::make_shared<can_interfaces::srv::RegisterMsg::Request>();
  request->channel = channel;
  request->pgn = subscription.FrameId;
  request->pgnmask = subscription.FrameIdMask;
  request->mask = mask;
  request->match = match;
  auto result = RegisterMsgClient->async_send_request(request);
  if (rclcpp::spin_until_future_complete(Node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    subscription.Handle = result.get()->handle;
  } else {
    RCLCPP_ERROR(Node->get_logger(), "Failed to call service register_msg");
  }
  Subscriptions.insert(SubscriptionPair(pgn, subscription));
  return subscription;
}

CanBridge::Subscription_t CanBridge::Register(uint8_t channel, int32_t pgn, uint8_t address, const ForwarderFn& fn)
{
  return Register(channel, pgn, 0, 0, address, fn);
}

CanBridge::Subscription_t CanBridge::Register1Cmd(
  uint8_t channel, int32_t pgn, uint8_t cmdByte,
  uint8_t address, const ForwarderFn& fn)
{
  uint64_t mask = 0xFF;
  uint64_t match = cmdByte;
  return Register(channel, pgn, mask, match, address, fn);
}

CanBridge::Subscription_t CanBridge::Register2Cmd(
  uint8_t channel, int32_t pgn, uint8_t cmdByte1, uint8_t cmdByte2,
  uint8_t address, const ForwarderFn& fn)
{
  uint64_t mask = 0xFFFF;
  uint64_t match = (cmdByte2 << 8) | cmdByte1;
  return Register(channel, pgn, mask, match, address, fn);
}

CanBridge::ForwarderFn CanBridge::MakeGenericForwarder(const char* topic)
{
	auto publisher = Node->create_publisher<can_interfaces::msg::GenericMsg>(topic, 10);//TODO What QoS do we need?
	return [publisher](const sc::CanMessage_t& msg){
		can_interfaces::msg::GenericMsg rosMsg;
		rosMsg.can_data.frame_id = msg.FrameId;
		rosMsg.can_data.timestamp = msg.Timestamp;
		rosMsg.length = msg.Length;
		memcpy(rosMsg.data.data(), msg.Bytes, msg.Length);
		publisher->publish(rosMsg);
	};
}

CanBridge::Subscription_t CanBridge::GenericRegister(uint8_t channel, int32_t pgn, uint64_t mask, uint64_t match, uint8_t address, const char* topic)
{
	return Register(channel, pgn, mask, match, address, MakeGenericForwarder(topic));
}

CanBridge::Subscription_t CanBridge::GenericRegister(uint8_t channel, int32_t pgn, uint8_t address, const char* topic)
{
	return Register(channel, pgn, address, MakeGenericForwarder(topic));
}

CanBridge::Subscription_t CanBridge::GenericRegister1Cmd(uint8_t channel, int32_t pgn, uint8_t cmdByte, uint8_t address, const char* topic)
{
	return Register1Cmd(channel, pgn, cmdByte, address, MakeGenericForwarder(topic));
}

CanBridge::Subscription_t CanBridge::GenericRegister2Cmd(uint8_t channel, int32_t pgn, uint8_t cmdByte1, uint8_t cmdByte2, uint8_t address, const char* topic)
{
	return Register2Cmd(channel, pgn, cmdByte1, cmdByte2, address, MakeGenericForwarder(topic));
}

bool CanBridge::Matches(const Subscription_t & subscription, const sc::CanMessage_t & msg)
{
  return ( (subscription.FrameIdMask & msg.FrameId) == subscription.FrameId) &&
         ((subscription.Mask & *reinterpret_cast<const uint64_t *>(msg.Bytes)) ==
         subscription.Match);
}

void CanBridge::HandleMessage(const sc::CanMessage_t & msg)
{
  uint32_t maskedPgn = (msg.FrameId & PGN_MASK)>>8;
  auto range = Subscriptions.equal_range(maskedPgn);
  for (auto it = range.first; it != range.second; ++it) {
    if (Matches(it->second, msg)) {
      it->second.Fn(msg);
    }
  }
}

void CanBridge::Send(const sc::CanMessage_t& out)
{
    if (!TxQueue->try_send(&out, sizeof(sc::CanMessage_t), 0)) {
      std::cout << "CAN Send Failed" << std::endl;
    }
}