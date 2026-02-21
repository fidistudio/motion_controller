import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from robot_interfaces.msg import Force2D

import math


class ProportionalVelocityController(Node):

    def __init__(self) -> None:
        super().__init__("proportional_velocity_controller")

        # Control parameters
        self.declare_parameter("angular_gain", 4.0)
        self.declare_parameter("orientation_tolerance", math.pi / 10)
        self.declare_parameter("goal_vector_tolerance", 1e-6)
        self.declare_parameter("max_linear_velocity", 2.5)

        # State
        self._pose: Pose2D | None = None
        self._descent_vector: Force2D | None = None

        # Publisher
        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        # Subscriptions
        self.create_subscription(Pose2D, "/robot_pose", self._on_pose, 10)
        self.create_subscription(
            Force2D, "/gradient_descent_vector", self._on_descent_vector, 10
        )

        # 20 Hz control loop
        self.create_timer(0.05, self._control_loop)

        self.get_logger().info("Velocity controller initialized")

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_pose(self, msg: Pose2D) -> None:
        self._pose = msg

    def _on_descent_vector(self, msg: Force2D) -> None:
        self._descent_vector = msg

    # ------------------------------------------------------------------
    # Control loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if self._pose is None or self._descent_vector is None:
            return

        angular_gain = self.get_parameter("angular_gain").value
        orientation_tol = self.get_parameter("orientation_tolerance").value
        goal_vector_tol = self.get_parameter("goal_vector_tolerance").value
        max_linear_vel = self.get_parameter("max_linear_velocity").value

        direction_angle = self._desired_heading()
        heading_error = self._angle_error(self._pose.theta, direction_angle)
        vector_magnitude = self._descent_vector_magnitude()

        cmd = Twist()

        if vector_magnitude < goal_vector_tol:
            cmd = self._stop_command()
            decision = "STOP (goal tolerance reached)"

        elif abs(heading_error) > orientation_tol:
            cmd = self._rotate_in_place(angular_gain, heading_error)
            decision = "ROTATE (aligning with descent direction)"

        else:
            cmd = self._move_toward_target(
                angular_gain,
                heading_error,
                vector_magnitude,
                max_linear_vel,
            )
            decision = "MOVE (aligned and advancing)"

        self._cmd_pub.publish(cmd)

        # Throttled debug logging (every 1 second)
        self.get_logger().info(
            (
                f"[CONTROL] "
                f"|v|={vector_magnitude:.3f} "
                f"theta_d={direction_angle:.3f} "
                f"error={heading_error:.3f} "
                f"cmd_lin={cmd.linear.x:.3f} "
                f"cmd_ang={cmd.angular.z:.3f} "
                f"decision={decision}"
            ),
            throttle_duration_sec=1.0,
        )

    # ------------------------------------------------------------------
    # Control primitives
    # ------------------------------------------------------------------

    def _desired_heading(self) -> float:
        return math.atan2(
            self._descent_vector.fy,
            self._descent_vector.fx,
        )

    def _descent_vector_magnitude(self) -> float:
        return math.hypot(
            self._descent_vector.fx,
            self._descent_vector.fy,
        )

    def _rotate_in_place(self, gain: float, error: float) -> Twist:
        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.angular.z = gain * error
        return cmd

    def _move_toward_target(
        self,
        gain: float,
        error: float,
        magnitude: float,
        max_linear_vel: float,
    ) -> Twist:
        cmd = Twist()
        cmd.linear.x = min(magnitude, max_linear_vel)
        cmd.angular.z = gain * error
        return cmd

    @staticmethod
    def _stop_command() -> Twist:
        return Twist()

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    @staticmethod
    def _angle_error(theta: float, theta_desired: float) -> float:
        error = theta_desired - theta
        return math.atan2(math.sin(error), math.cos(error))


def main() -> None:
    rclpy.init()
    node = ProportionalVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
