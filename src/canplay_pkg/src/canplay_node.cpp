#include "CanPlayer.hpp"
#include "CanNode.hpp"

int main(int argc, char ** argv)
{
  if (argc > 1) {
    CanPlayer player;
    if (!player.Open(argv[1])) {
      std::cout << "Couldn't open file: " << argv[1] << std::endl;
    } else {
      rclcpp::init(argc, argv);
      auto node = rclcpp::Node::make_shared("canplay_node");
      CanNode canNode(player, node);
      while (canNode.Step()) {
        rclcpp::spin_some(node);
      }
      rclcpp::shutdown();
    }
  } else {
    std::cout << "canplay_node [filename]" << std::endl;
  }
  return 0;
}
