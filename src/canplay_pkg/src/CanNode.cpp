///////////////////////////////////////////////////////////////////////////////
// Copyright Deere & Company. For more information,
// please see COPYRIGHT file in root of source repository.
///////////////////////////////////////////////////////////////////////////////
#include "CanNode.hpp"
#include <CANFactory/CanQueues.hpp>
#include "ProcessingFns.hpp"
#include <iostream>
#include <thread>

using namespace boost::interprocess;
using namespace std::chrono;

CanNode::CanNode(ICanSource & source, std::shared_ptr<rclcpp::Node> & node)
: Node(node)
, Source(source)
, StartTime(steady_clock::now())
{
  CreateMessageQueues();
  RegisterMsgSvc = Node->create_service<can_interfaces::srv::RegisterMsg>(
    "register_msg", [this](
      const std::shared_ptr<can_interfaces::srv::RegisterMsg::Request> req,
      std::shared_ptr<can_interfaces::srv::RegisterMsg::Response> resp) {
      resp->set__handle(
        AddSubscription(
          req->channel, req->pgnmask, req->pgn, req->mask,
          req->match));
    });
}

int32_t CanNode::AddSubscription( uint8_t channel, int32_t pgnMask, int32_t pgn, int64_t mask, int64_t match)
{
  Subscription_t subscription;
  subscription.Handle = ++LastHandle;
  subscription.Channel = channel;
  subscription.PGN = pgn;
  subscription.PGNMask = pgnMask;
  subscription.Mask = mask;
  subscription.Match = match;
  uint32_t maskedPgn = pgn & PGN_MASK;
  Subscriptions.insert(SubscriptionPair(maskedPgn, subscription));
  std::cout << "Added Subscription pgn:" << std::hex  << std::uppercase << pgn << " pgnMask:"<< pgnMask << std::endl;
  return subscription.Handle;
}

std::shared_ptr<boost::interprocess::message_queue> CanNode::MakeRxQueue()
{
  std::shared_ptr<message_queue> queue;
  do {
    try {
      message_queue::remove(sc::CanRx);
      queue = std::make_shared<message_queue>(create_only, sc::CanRx, sc::RxSize, sizeof(sc::CanMessage_t));
    } catch (const std::exception & e) {
      std::cout << "Couldn't create the canbus rx queue..." << std::endl;
      std::cerr << e.what() << '\n';
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
  } while (!queue);
  return queue;
}

std::shared_ptr<boost::interprocess::message_queue> CanNode::MakeTxQueue()
{
  std::shared_ptr<message_queue> queue;
  try {
    message_queue::remove(sc::CanTx);
    queue = std::make_shared<message_queue>(create_only, sc::CanTx, sc::TxSize, sizeof(sc::CanMessage_t));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "rclcpp"), "Couldn't create the canbus tx queue...");
    std::cerr << e.what() << '\n';
  }
  return queue;
}

void CanNode::CreateMessageQueues()
{
  TxQueue = MakeTxQueue();
  RxQueue = MakeRxQueue();
}

bool CanNode::Matches(const Subscription_t & subscription, const sc::CanMessage_t & msg)
{
  return ( (subscription.PGNMask & msg.FrameId) == subscription.PGN) &&
         ((subscription.Mask & *reinterpret_cast<const uint64_t *>(msg.Bytes)) ==
         subscription.Match);
}

bool CanNode::HasSubscription(const sc::CanMessage_t & msg) const
{
  bool hasSubscription = false;
  uint32_t maskedPgn = msg.FrameId & PGN_MASK;
  auto range = Subscriptions.equal_range(maskedPgn);
  for (auto it = range.first; it != range.second; ++it) {
    if (Matches(it->second, msg)) {
      hasSubscription = true;
      break;
    }
  }
  return hasSubscription;
}

void CanNode::Publish(const sc::CanMessage_t & msg)
{
  if (HasSubscription(msg)) {
    sc::CanMessage_t timestampedMsg = msg;
    timestampedMsg.Timestamp = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
    if (!RxQueue->try_send(&timestampedMsg, sizeof(sc::CanMessage_t), 0)) {
      std::cout << "Send Failed" << std::endl;
    }
  }
}

bool CanNode::PopOutgoing(sc::CanMessage_t& msg)
{
  bool gotMsg = false;
  std::size_t receivedSize;
  unsigned int priority = 0;
  if (TxQueue->try_receive(&msg, sizeof(sc::CanMessage_t), receivedSize, priority)) {
    if (receivedSize == sizeof(sc::CanMessage_t)) {
      gotMsg = true;
    }
  }
  return gotMsg;
}

double CanNode::GetTimestamp()
{
  return duration_cast<milliseconds>((steady_clock::now() - StartTime)).count()/1000.0;
}

bool CanNode::Step()
{
  bool more = false;
  bool isValid = false;
  MessageInstance_t msg;
  if (Source.TryReceive(isValid, msg)) {
    more = true;
    if (isValid) {
      Publish(msg.Msg);
      //std::cout << "rx " << sc::ToString(msg.Msg, GetTimestamp()) << std::endl;
    }
  }
  sc::CanMessage_t outgoingMsg;
  while(PopOutgoing(outgoingMsg))
  {
    Source.Send(outgoingMsg);
    std::cout << "tx " << sc::ToString(outgoingMsg, GetTimestamp()) << std::endl;
  }
  return more;
}
