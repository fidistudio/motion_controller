#include "motion_controller/velocity_command_computer.hpp"

#include <algorithm>
#include <cmath>

namespace motion_controller {

geometry_msgs::msg::Twist
VelocityCommandComputer::compute(double fx, double fy,
                                 const VelocityControllerParams &params) {
  const double angle_error = std::atan2(fy, fx);
  const double magnitude = std::hypot(fx, fy);

  // Linear velocity: proportional to magnitude, scaled by alignment with
  // robot's forward axis. cos(angle_error) ∈ [-1, 1] — clamped to [0, 1]
  // so the robot never drives backwards.
  const double alignment = std::max(std::cos(angle_error), 0.0);
  const double linear = params.linear_gain * magnitude * alignment;

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = clamp(linear, 0.0, params.max_linear_velocity);
  cmd.angular.z =
      clamp(params.angular_gain * angle_error, -params.max_angular_velocity,
            params.max_angular_velocity);

  return cmd;
}

double VelocityCommandComputer::clamp(double value, double min_val,
                                      double max_val) {
  return std::clamp(value, min_val, max_val);
}

} // namespace motion_controller
