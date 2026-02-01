import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose2D
from robot_interfaces.msg import Force2D
import math


class VelocityController(Node):

    def __init__(self):
        super().__init__("velocity_controller")

        self.declare_parameter("k_theta", 4.0)
        self.declare_parameter("eps_orient", math.pi / 10)
        self.declare_parameter("eps_goal", 0.8)
        self.declare_parameter("v_max", 2.5)

        self.pose = None
        self.force = None

        self.cmd_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        self.create_subscription(Pose2D, "/robot_pose", self.pose_callback, 10)
        self.create_subscription(Force2D, "/force_vector", self.force_callback, 10)

        self.timer = self.create_timer(0.05, self.control_loop)

    def pose_callback(self, msg: Pose2D) -> None:
        self.pose = msg

    def force_callback(self, msg: Force2D) -> None:
        self.force = msg

    def control_loop(self) -> None:
        if self.pose is None or self.force is None:
            return

        theta_d = math.atan2(self.force.fy, self.force.fx)
        error = self._angle_error(self.pose.theta, theta_d)

        force_norm = math.hypot(self.force.fx, self.force.fy)

        cmd = Twist()

        if force_norm < self.get_parameter("eps_goal").value:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif abs(error) > self.get_parameter("eps_orient").value:
            cmd.linear.x = 0.0
            cmd.angular.z = self.get_parameter("k_theta").value * error
        else:
            cmd.linear.x = min(force_norm, self.get_parameter("v_max").value)
            cmd.angular.z = self.get_parameter("k_theta").value * error

        self.cmd_pub.publish(cmd)

    @staticmethod
    def _angle_error(theta, theta_d):
        e = theta_d - theta
        while e > math.pi:
            e -= 2 * math.pi
        while e < -math.pi:
            e += 2 * math.pi
        return e


def main():
    rclpy.init()
    node = VelocityController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
