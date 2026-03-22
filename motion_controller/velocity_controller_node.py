import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from robot_interfaces.msg import Force2D

import math


class ProportionalVelocityController(Node):

    def __init__(self) -> None:
        super().__init__("proportional_velocity_controller")

        self._declare_parameters()

        self._descent_vector: Force2D | None = None

        self._cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(
            Force2D,
            "/gradient_descent_vector",
            self._on_descent_vector,
            10,
        )

        self.create_timer(0.05, self._control_loop)

        self.get_logger().info("Velocity controller initialized")

    # ------------------------------------------------------------------
    # Initialization
    # ------------------------------------------------------------------

    def _declare_parameters(self) -> None:
        self.declare_parameter("angular_gain", 1.0)
        self.declare_parameter("linear_gain", 1.0)
        self.declare_parameter("goal_vector_tolerance", 1e-6)
        self.declare_parameter("max_linear_velocity", 0.5)
        self.declare_parameter("max_angular_velocity", 3.0)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------

    def _on_descent_vector(self, msg: Force2D) -> None:
        self._descent_vector = msg

    # ------------------------------------------------------------------
    # Control Loop
    # ------------------------------------------------------------------

    def _control_loop(self) -> None:
        if self._descent_vector is None:
            return

        fx = self._descent_vector.fx  # ya en base_footprint
        fy = self._descent_vector.fy

        magnitude = math.hypot(fx, fy)
        tolerance = self.get_parameter("goal_vector_tolerance").value

        if magnitude < tolerance:
            self._publish_zero()
            return

        cmd = self._compute_command(fx, fy)
        self._cmd_pub.publish(cmd)

    # ------------------------------------------------------------------
    # Command Computation
    # ------------------------------------------------------------------

    def _compute_command(self, fx: float, fy: float) -> Twist:
        linear_gain = self.get_parameter("linear_gain").value
        angular_gain = self.get_parameter("angular_gain").value
        max_linear = self.get_parameter("max_linear_velocity").value
        max_angular = self.get_parameter("max_angular_velocity").value

        # Ángulo del vector respecto al eje X del robot (eje frontal)
        angle_error = math.atan2(fy, fx)

        # Velocidad lineal proporcional a qué tan alineado está el robot
        # cos(angle_error): 1 si apunta recto, 0 si es perpendicular, nunca negativo
        magnitude = math.hypot(fx, fy)
        alignment = math.cos(angle_error)  # ∈ [-1, 1]
        linear = linear_gain * magnitude * max(alignment, 0.0)

        cmd = Twist()
        cmd.linear.x = self._clamp(linear, 0.0, max_linear)  # solo hacia adelante
        cmd.angular.z = self._clamp(
            angular_gain * angle_error, -max_angular, max_angular
        )
        return cmd

    # ------------------------------------------------------------------
    # Utilities
    # ------------------------------------------------------------------

    def _publish_zero(self) -> None:
        self._cmd_pub.publish(Twist())

    @staticmethod
    def _clamp(value: float, min_value: float, max_value: float) -> float:
        return max(min(value, max_value), min_value)


def main() -> None:
    rclpy.init()
    node = ProportionalVelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
