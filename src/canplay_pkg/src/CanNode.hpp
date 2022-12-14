///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#pragma once
#include "ICanSource.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/interprocess/ipc/message_queue.hpp>
#include <can_interfaces/srv/register_msg.hpp>
#include <chrono>
#include <map>

class CanNode
{
private:
  static constexpr uint32_t INVALID_HANDLE = 0;
  static constexpr uint32_t PGN_MASK = 0x00FFFF00;
  struct Subscription_t
  {
    uint32_t Handle = INVALID_HANDLE;
    uint8_t Channel = 0;
    uint32_t PGN = 0;
    uint32_t PGNMask = 0;
    uint64_t Mask = 0;
    uint64_t Match = 0;
  };
  using SubscriptionMultimap = std::multimap<uint32_t, Subscription_t>;
  using SubscriptionPair = std::pair<uint32_t, Subscription_t>;
  using SubscriptionIterator = SubscriptionMultimap::iterator;
  using SubscriptionRange = std::pair<SubscriptionIterator, SubscriptionIterator>;

  std::shared_ptr<rclcpp::Node> & Node;
  ICanSource & Source;
  std::shared_ptr<boost::interprocess::message_queue> RxQueue;
  std::shared_ptr<boost::interprocess::message_queue> TxQueue;
  rclcpp::Service<can_interfaces::srv::RegisterMsg>::SharedPtr RegisterMsgSvc;
  SubscriptionMultimap Subscriptions;
  uint32_t LastHandle = 0;
  std::chrono::_V2::steady_clock::time_point StartTime;

  static std::shared_ptr<boost::interprocess::message_queue> MakeRxQueue();
  static std::shared_ptr<boost::interprocess::message_queue> MakeTxQueue();

  static bool Matches(const Subscription_t & subscription, const sc::CanMessage_t & msg);
  void CreateMessageQueues();
  bool HasSubscription(const sc::CanMessage_t & msg) const;
  bool Publish(const sc::CanMessage_t & msg);
  int32_t AddSubscription(uint8_t channel, int32_t pgnMask, int32_t pgn, int64_t mask,int64_t match);
  bool PopOutgoing(sc::CanMessage_t& msg);
  double GetTimestamp();
public:
  CanNode(ICanSource & source, std::shared_ptr<rclcpp::Node> & node);
  bool Step();
};
