#pragma once
#include "can_interfaces/srv/register_msg.hpp"
#include "rclcpp/rclcpp.hpp"
#include <boost/interprocess/ipc/message_queue.hpp>
#include "std_msgs/msg/string.hpp"

class CanBridge
{
  private:
    static constexpr uint32_t PGN_MASK = 0x00FFFF00;

    std::shared_ptr<rclcpp::Node>& Node;
    std::shared_ptr<boost::interprocess::message_queue> Queue;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr Publish;
    rclcpp::Client<can_interfaces::srv::RegisterMsg>::SharedPtr RegisterMsgClient;
    rclcpp::TimerBase::SharedPtr Timer;

    size_t Count = 0;

    static std::shared_ptr<boost::interprocess::message_queue> MakeQueue();

    void WaitForService();
    void OnTimeout();

  public:
    static constexpr uint32_t INVALID_HANDLE = 0;
    static constexpr uint8_t NULL_ADDRESS = 0xFE;

    explicit CanBridge(std::shared_ptr<rclcpp::Node> & node);
    uint32_t Register( uint8_t channel, int32_t pgn, uint64_t mask, uint64_t match, uint8_t address = NULL_ADDRESS);
    uint32_t Register( uint8_t channel, int32_t pgn, uint8_t address = NULL_ADDRESS);
    uint32_t Register1Cmd( uint8_t channel, int32_t pgn, uint8_t cmdByte, uint8_t address = NULL_ADDRESS);
    uint32_t Register2Cmd( uint8_t channel, int32_t pgn, uint8_t cmdByte1, uint8_t cmdByte2, uint8_t address = NULL_ADDRESS);
    void Step();
};