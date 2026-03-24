#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from std_srvs.srv import SetBool


class LidControlNode(Node):
    """
    Controls the prismatic lid joint via Gazebo's native JointPositionController.
    Publishes desired position as Float64 to /lid_position (bridged to Gazebo).
    Provides /lid_control service (SetBool: true=open, false=close).
    """

    LID_OPEN = 0.18      # meters (URDF upper limit)
    LID_CLOSED = 0.0     # meters (URDF lower limit)

    def __init__(self):
        super().__init__('lid_control_node')

        # Publisher to Gazebo JointPositionController via bridge
        self.pub = self.create_publisher(Float64, '/lid_position', 10)

        # Service: SetBool (true = open, false = close)
        self.srv = self.create_service(
            SetBool, '/lid_control', self.lid_service_cb)

        # Publish closed position initially, keep publishing at 2 Hz
        self.target = self.LID_CLOSED
        self.timer = self.create_timer(0.5, self.publish_position)

        self.get_logger().info('Lid Control Node started. Service: /lid_control')

    def lid_service_cb(self, request, response):
        if request.data:
            self.target = self.LID_OPEN
            response.message = 'Lid opened'
            self.get_logger().info('Lid OPENED via service')
        else:
            self.target = self.LID_CLOSED
            response.message = 'Lid closed'
            self.get_logger().info('Lid CLOSED via service')
        response.success = True
        return response

    def publish_position(self):
        msg = Float64()
        msg.data = self.target
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LidControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
