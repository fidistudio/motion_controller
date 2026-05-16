#include "motion_controller/proportional_velocity_controller_node.hpp"

#include "rclcpp/rclcpp.hpp"

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<
               motion_controller::ProportionalVelocityControllerNode>());
  rclcpp::shutdown();
  return 0;
}
