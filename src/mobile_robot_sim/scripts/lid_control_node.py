#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import SetBool


class LidControlNode(Node):
    """
    Controls the prismatic lid joint.
    - Publishes position commands to /lid_controller/commands
    - Provides a ROS service /lid_control (SetBool: true=open, false=close)
    - Also subscribes to /lid_command topic (Float64MultiArray) for direct control
    """

    def __init__(self):
        super().__init__('lid_control_node')

        self.lid_open_pos = 0.18    # fully open (meters)
        self.lid_closed_pos = 0.0   # fully closed

        # Publisher to the position controller
        self.pub = self.create_publisher(
            Float64MultiArray, '/lid_controller/commands', 10)

        # Service: SetBool (true = open, false = close)
        self.srv = self.create_service(
            SetBool, '/lid_control', self.lid_service_callback)

        # Topic-based control
        self.sub = self.create_subscription(
            Float64MultiArray, '/lid_command', self.lid_topic_callback, 10)

        self.get_logger().info('Lid Control Node started. Service: /lid_control')

    def lid_service_callback(self, request, response):
        if request.data:
            self._publish_position(self.lid_open_pos)
            response.success = True
            response.message = 'Lid opened'
            self.get_logger().info('Lid OPENED via service')
        else:
            self._publish_position(self.lid_closed_pos)
            response.success = True
            response.message = 'Lid closed'
            self.get_logger().info('Lid CLOSED via service')
        return response

    def lid_topic_callback(self, msg: Float64MultiArray):
        if len(msg.data) > 0:
            pos = max(self.lid_closed_pos, min(msg.data[0], self.lid_open_pos))
            self._publish_position(pos)
            self.get_logger().info(f'Lid position set to {pos:.3f} via topic')

    def _publish_position(self, position: float):
        cmd = Float64MultiArray()
        cmd.data = [position]
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = LidControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
