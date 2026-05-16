#pragma once

#include "motion_controller/velocity_command_computer.hpp"

#include <optional>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "robot_interfaces/msg/force2_d.hpp"

namespace motion_controller {

/**
 * @brief ROS 2 node: proportional velocity controller.
 *
 * Pipeline per timer tick:
 *   1. Guard: field vector must have been received.
 *   2. Check vector magnitude against tolerance → publish zero Twist if below.
 *   3. Delegate command computation to VelocityCommandComputer.
 *   4. Publish resulting Twist on /cmd_vel.
 */
class ProportionalVelocityControllerNode : public rclcpp::Node {
public:
  explicit ProportionalVelocityControllerNode(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions{});

private:
  // ── Callbacks ──────────────────────────────────────────────────────────────
  void onDescentVector(const robot_interfaces::msg::Force2D &msg);
  void controlLoop();

  // ── Helpers ────────────────────────────────────────────────────────────────
  [[nodiscard]] VelocityControllerParams readParams() const;
  void publishZero();

  // ── State ──────────────────────────────────────────────────────────────────
  std::optional<robot_interfaces::msg::Force2D> descent_vector_;

  // ── ROS interfaces ─────────────────────────────────────────────────────────
  rclcpp::Subscription<robot_interfaces::msg::Force2D>::SharedPtr vector_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

} // namespace motion_controller
