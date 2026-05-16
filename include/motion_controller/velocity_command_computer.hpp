#pragma once

#include "geometry_msgs/msg/twist.hpp"

namespace motion_controller {

/**
 * @brief Parameters for the proportional velocity controller.
 */
struct VelocityControllerParams {
  double angular_gain{1.0};
  double linear_gain{1.0};
  double max_linear_velocity{0.5};
  double max_angular_velocity{3.0};
};

/**
 * @brief Pure math class: converts a 2D force vector into a Twist command.
 *
 * No ROS node references — only geometry and control law.
 * Unit-testable without spinning a ROS context.
 *
 * Control law:
 *   - angle_error  = atan2(fy, fx)          [rad, error w.r.t. robot forward
 * axis]
 *   - linear.x     = k_lin * |f| * cos(angle_error)  clamped to [0, max_linear]
 *   - angular.z    = k_ang * angle_error              clamped to [-max_angular,
 * max_angular]
 */
class VelocityCommandComputer {
public:
  /**
   * @brief Compute a Twist from a 2D field vector.
   *
   * @param fx      X component of the field vector in base_footprint
   * [m/s²-ish].
   * @param fy      Y component of the field vector in base_footprint
   * [m/s²-ish].
   * @param params  Controller gains and saturation limits.
   * @return        Twist command ready to publish on /cmd_vel.
   */
  [[nodiscard]] static geometry_msgs::msg::Twist
  compute(double fx, double fy, const VelocityControllerParams &params);

private:
  [[nodiscard]] static double clamp(double value, double min_val,
                                    double max_val);
};

} // namespace motion_controller
