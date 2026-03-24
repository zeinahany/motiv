#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Float64MultiArray
import math


class MecanumDriveNode(Node):
    """
    Subscribes to /cmd_vel (Twist or TwistStamped).
    Converts (vx, vy, wz) to 4 individual wheel velocities using mecanum kinematics.
    Publishes wheel velocity commands.
    """

    def __init__(self):
        super().__init__('mecanum_drive_node')

        # Robot parameters
        self.wheel_radius = 0.06  # meters
        # lx + ly = half-wheelbase-x + half-wheelbase-y = 0.1975 + 0.236 = 0.4335
        self.lx_plus_ly = 0.4335

        # Subscribe to cmd_vel (TwistStamped for mecanum_drive_controller compatibility)
        self.sub_stamped = self.create_subscription(
            TwistStamped, '/mecanum_drive_controller/cmd_vel',
            self.cmd_vel_stamped_callback, 10)

        # Also subscribe to plain Twist on /cmd_vel for external tools
        self.sub_twist = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # Publish individual wheel velocity commands
        self.pub_fl = self.create_publisher(Float64MultiArray, '/fl_wheel_controller/commands', 10)
        self.pub_fr = self.create_publisher(Float64MultiArray, '/fr_wheel_controller/commands', 10)
        self.pub_rl = self.create_publisher(Float64MultiArray, '/rl_wheel_controller/commands', 10)
        self.pub_rr = self.create_publisher(Float64MultiArray, '/rr_wheel_controller/commands', 10)

        # Forward plain cmd_vel as stamped to the mecanum controller
        self.pub_cmd_stamped = self.create_publisher(
            TwistStamped, '/mecanum_drive_controller/cmd_vel', 10)

        self.get_logger().info('Mecanum Drive Node started.')

    def cmd_vel_callback(self, msg: Twist):
        """Convert plain Twist to TwistStamped and forward."""
        stamped = TwistStamped()
        stamped.header.stamp = self.get_clock().now().to_msg()
        stamped.header.frame_id = 'RobotBody'
        stamped.twist = msg
        self.pub_cmd_stamped.publish(stamped)
        self._compute_and_publish(msg.linear.x, msg.linear.y, msg.angular.z)

    def cmd_vel_stamped_callback(self, msg: TwistStamped):
        """Handle TwistStamped directly."""
        self._compute_and_publish(
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.angular.z)

    def _compute_and_publish(self, vx: float, vy: float, wz: float):
        """
        Mecanum inverse kinematics:
          w_fl = (1/r) * (vx - vy - (lx+ly)*wz)
          w_fr = (1/r) * (vx + vy + (lx+ly)*wz)
          w_rl = (1/r) * (vx + vy - (lx+ly)*wz)
          w_rr = (1/r) * (vx - vy + (lx+ly)*wz)
        """
        r = self.wheel_radius
        k = self.lx_plus_ly

        w_fl = (1.0 / r) * (vx - vy - k * wz)
        w_fr = (1.0 / r) * (vx + vy + k * wz)
        w_rl = (1.0 / r) * (vx + vy - k * wz)
        w_rr = (1.0 / r) * (vx - vy + k * wz)

        for pub, vel in [(self.pub_fl, w_fl), (self.pub_fr, w_fr),
                         (self.pub_rl, w_rl), (self.pub_rr, w_rr)]:
            cmd = Float64MultiArray()
            cmd.data = [vel]
            pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MecanumDriveNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
