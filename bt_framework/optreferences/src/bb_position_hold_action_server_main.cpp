#include "optreferences/bb_position_hold_action_server.hpp"

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<optreferences::BbPositionHoldActionServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}