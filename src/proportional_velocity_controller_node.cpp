#include "motion_controller/proportional_velocity_controller_node.hpp"

#include <cmath>

namespace motion_controller {

// ── Constructor
// ───────────────────────────────────────────────────────────────

ProportionalVelocityControllerNode::ProportionalVelocityControllerNode(
    const rclcpp::NodeOptions &options)
    : Node("proportional_velocity_controller", options) {
  declare_parameter("vector_topic", "/gradient_descent_vector");
  declare_parameter("cmd_topic", "/cmd_vel");
  declare_parameter("goal_vector_tolerance", 1e-6);
  declare_parameter("angular_gain", 1.0);
  declare_parameter("linear_gain", 1.0);
  declare_parameter("max_linear_velocity", 0.5);
  declare_parameter("max_angular_velocity", 3.0);

  const auto vector_topic = get_parameter("vector_topic").as_string();
  const auto cmd_topic = get_parameter("cmd_topic").as_string();

  vector_sub_ = create_subscription<robot_interfaces::msg::Force2D>(
      vector_topic, 10, [this](const robot_interfaces::msg::Force2D &msg) {
        onDescentVector(msg);
      });

  cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_topic, 10);

  timer_ = create_wall_timer(std::chrono::milliseconds(50),
                             [this]() { controlLoop(); });

  RCLCPP_INFO(get_logger(), "ProportionalVelocityControllerNode ready");
}

// ── Callbacks
// ─────────────────────────────────────────────────────────────────

void ProportionalVelocityControllerNode::onDescentVector(
    const robot_interfaces::msg::Force2D &msg) {
  descent_vector_ = msg;
}

void ProportionalVelocityControllerNode::controlLoop() {
  if (!descent_vector_.has_value()) {
    return;
  }

  const double fx = static_cast<double>(descent_vector_->fx);
  const double fy = static_cast<double>(descent_vector_->fy);

  const double tolerance = get_parameter("goal_vector_tolerance").as_double();

  if (std::hypot(fx, fy) < tolerance) {
    publishZero();
    return;
  }

  const auto cmd = VelocityCommandComputer::compute(fx, fy, readParams());
  cmd_pub_->publish(cmd);
}

// ── Helpers
// ───────────────────────────────────────────────────────────────────

VelocityControllerParams
ProportionalVelocityControllerNode::readParams() const {
  return VelocityControllerParams{
      get_parameter("angular_gain").as_double(),
      get_parameter("linear_gain").as_double(),
      get_parameter("max_linear_velocity").as_double(),
      get_parameter("max_angular_velocity").as_double()};
}

void ProportionalVelocityControllerNode::publishZero() {
  cmd_pub_->publish(geometry_msgs::msg::Twist{});
}

} // namespace motion_controller
