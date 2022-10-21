#include "CanBridge.hpp"
#include <CANFactory/CanMessage.h>
#include <chrono>
using namespace std::chrono_literals;
using namespace boost::interprocess;

CanBridge::CanBridge(std::shared_ptr<rclcpp::Node> & node)
: Node(node),
  Queue(CanBridge::MakeQueue()),
  Publish(Node->create_publisher<std_msgs::msg::String>("topic", 10)),
  RegisterMsgClient(Node->create_client<can_interfaces::srv::RegisterMsg>("register_msg")),
  Timer(Node->create_wall_timer(500ms, std::bind(&CanBridge::OnTimeout, this)))
{
  WaitForService();
}

std::shared_ptr<message_queue> CanBridge::MakeQueue()
{
  std::shared_ptr<message_queue> queue;
  try {
    message_queue::remove("canbus");
    queue = std::make_shared<message_queue>(create_only, "canbus", 1000, sizeof(sc::CanMessage_t));
  } catch (const std::exception & /*e*/) {
    RCLCPP_ERROR(
      rclcpp::get_logger(
        "rclcpp"), "Couldn't create the canbus queue...");
    //std::cerr << e.what() << '\n';
  }
  return queue;
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
  if (Queue->try_receive(&msg, sizeof(sc::CanMessage_t), receivedSize, priority)) {
    if (receivedSize == sizeof(sc::CanMessage_t)) {
      Count++;
    }
  }
}

uint32_t CanBridge::Register(
  uint8_t channel, int32_t pgn, uint64_t mask, uint64_t match,
  uint8_t address)
{
  uint32_t handle = INVALID_HANDLE;
  auto request = std::make_shared<can_interfaces::srv::RegisterMsg::Request>();
  request->channel = channel;
  request->pgn = address == NULL_ADDRESS ? (pgn << 8) : ((pgn << 8) | address);
  request->pgnmask = address == NULL_ADDRESS ? 0xFFFF00 : 0xFFFFFF;
  request->mask = mask;
  request->match = match;
  auto result = RegisterMsgClient->async_send_request(request);
  if (rclcpp::spin_until_future_complete(Node, result) == rclcpp::FutureReturnCode::SUCCESS) {
    handle = result.get()->handle;
  } else {
    RCLCPP_ERROR(Node->get_logger(), "Failed to call service register_msg");
  }
  return handle;
}

uint32_t CanBridge::Register(uint8_t channel, int32_t pgn, uint8_t address)
{
  return Register(channel, pgn, 0, 0, address);
}

uint32_t CanBridge::Register1Cmd(uint8_t channel, int32_t pgn, uint8_t cmdByte, uint8_t address)
{
  uint64_t mask = 0xFF;
  uint64_t match = cmdByte;
  return Register(channel, pgn, mask, match, address);
}

uint32_t CanBridge::Register2Cmd(
  uint8_t channel, int32_t pgn, uint8_t cmdByte1, uint8_t cmdByte2,
  uint8_t address)
{
  uint64_t mask = 0xFFFF;
  uint64_t match = (cmdByte2 << 8) | cmdByte1;
  return Register(channel, pgn, mask, match, address);
}
