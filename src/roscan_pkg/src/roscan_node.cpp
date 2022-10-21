#include "CanBridge.hpp"
#include <GpsService/GpsServiceMessages.hpp>
#include <CANFactory/CanMessage.h>

int main(int argc, char * argv[])
{
    auto ros = bridge::BearingSpeed::ToROS(gps::BearingSpeed{1.0f,2.0f});
    auto cpp = bridge::BearingSpeed::FromROS(ros);
    assert(cpp.Bearing==1.0f);
    auto canMsg = sc::ToCanMessage<bridge::BearingSpeed>(cpp,1,0x26);
    (void)canMsg;
    std::cout << "0x" << std::hex << std::uppercase << canMsg.FrameId << std::endl;
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("roscan_node");
    CanBridge bridge(node);
    bridge.Register(2,0xFE15,0xF0);
    while(true)
    {
      bridge.Step();
      rclcpp::spin_some(node);
    }
    rclcpp::shutdown();
  return 0;
}