#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_srvs.srv import SetBool


class StartStopNode(Node):
    """
    Provides a /robot_enable service (SetBool).
    - true: enable the robot (allow cmd_vel pass-through)
    - false: disable the robot (publish zero velocity, block commands)

    Sits between goal_sender / obstacle_avoidance and the mecanum controller.
    Subscribes to /cmd_vel_input and republishes on /mecanum_drive_controller/cmd_vel
    only when enabled.
    """

    def __init__(self):
        super().__init__('start_stop_node')

        self.enabled = False

        # Service
        self.srv = self.create_service(
            SetBool, '/robot_enable', self.enable_callback)

        # Input velocity (from goal sender or obstacle avoidance)
        self.sub = self.create_subscription(
            TwistStamped, '/cmd_vel_input',
            self.cmd_vel_callback, 10)

        # Output velocity to controller
        self.pub = self.create_publisher(
            TwistStamped, '/mecanum_drive_controller/cmd_vel', 10)

        # Safety timer: if disabled, keep publishing zero
        self.timer = self.create_timer(0.1, self.safety_loop)

        self.get_logger().info('Start/Stop Node started. Service: /robot_enable')
        self.get_logger().info('Call: ros2 service call /robot_enable std_srvs/srv/SetBool "{data: true}"')

    def enable_callback(self, request, response):
        self.enabled = request.data
        state = 'ENABLED' if self.enabled else 'DISABLED'
        self.get_logger().info(f'Robot {state}')
        response.success = True
        response.message = f'Robot {state}'
        if not self.enabled:
            self._stop()
        return response

    def cmd_vel_callback(self, msg: TwistStamped):
        if self.enabled:
            self.pub.publish(msg)

    def safety_loop(self):
        if not self.enabled:
            self._stop()

    def _stop(self):
        cmd = TwistStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'RobotBody'
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = StartStopNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
