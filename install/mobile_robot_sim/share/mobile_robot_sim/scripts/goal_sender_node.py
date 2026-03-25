#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import math
import time


class GoalSenderNode(Node):
    """
    Sends the robot to a sequence of navigation goals using simple
    proportional control. Uses odometry feedback to determine when
    each goal is reached.
    """

    def __init__(self):
        super().__init__('goal_sender_node')

        # Navigation goals: (x, y, yaw_radians)
        self.goals = [
            (2.0, 0.0, 0.0),
            (2.0, 3.0, 1.5708),
            (-1.0, 3.0, 3.14159),
        ]
        self.current_goal_idx = 0
        self.goal_tolerance_pos = 0.25  # meters
        self.goal_tolerance_yaw = 0.15  # radians

        # Robot state from odometry
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        # Control gains
        self.kp_linear = 0.6
        self.kp_angular = 1.2
        self.kp_lateral = 0.6
        self.max_linear = 0.4
        self.max_angular = 0.6

        # Subscriber: odometry from EKF or Gazebo bridge
        self.odom_sub = self.create_subscription(
            Odometry, '/odometry/filtered', self.odom_callback, 10)

        # Fallback: also listen to Gazebo odom
        self.odom_sub2 = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)

        # Publisher: velocity commands
        self.cmd_pub = self.create_publisher(
            Twist, '/cmd_vel', 10)

        # Goal status publisher for visualization
        self.goal_pub = self.create_publisher(PoseStamped, '/current_goal', 10)

        # Control loop at 10 Hz
        self.timer = self.create_timer(0.1, self.control_loop)

        self.all_goals_done = False
        self.get_logger().info(f'Goal Sender started with {len(self.goals)} goals.')

    def odom_callback(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        # Extract yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.robot_yaw = math.atan2(siny_cosp, cosy_cosp)

    def control_loop(self):
        if self.all_goals_done:
            return

        if self.current_goal_idx >= len(self.goals):
            self.get_logger().info('All goals reached! Stopping.')
            self._stop()
            self.all_goals_done = True
            return

        gx, gy, g_yaw = self.goals[self.current_goal_idx]

        # Publish current goal for visualization
        goal_msg = PoseStamped()
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.header.frame_id = 'odom'
        goal_msg.pose.position.x = gx
        goal_msg.pose.position.y = gy
        goal_msg.pose.orientation.z = math.sin(g_yaw / 2.0)
        goal_msg.pose.orientation.w = math.cos(g_yaw / 2.0)
        self.goal_pub.publish(goal_msg)

        # Distance to goal
        dx = gx - self.robot_x
        dy = gy - self.robot_y
        dist = math.sqrt(dx * dx + dy * dy)
        yaw_err = self._normalize_angle(g_yaw - self.robot_yaw)

        # Check if goal reached
        if dist < self.goal_tolerance_pos and abs(yaw_err) < self.goal_tolerance_yaw:
            self.get_logger().info(
                f'Goal {self.current_goal_idx + 1} reached at '
                f'({self.robot_x:.2f}, {self.robot_y:.2f})')
            self.current_goal_idx += 1
            self._stop()
            return

        # Transform goal error into robot body frame
        cos_yaw = math.cos(self.robot_yaw)
        sin_yaw = math.sin(self.robot_yaw)
        local_x = dx * cos_yaw + dy * sin_yaw
        local_y = -dx * sin_yaw + dy * cos_yaw

        # Proportional control
        vx = self._clamp(self.kp_linear * local_x, -self.max_linear, self.max_linear)
        vy = self._clamp(self.kp_lateral * local_y, -self.max_linear, self.max_linear)
        wz = self._clamp(self.kp_angular * yaw_err, -self.max_angular, self.max_angular)

        # When close, focus on orientation
        if dist < self.goal_tolerance_pos * 2:
            vx *= 0.3
            vy *= 0.3

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = wz
        self.cmd_pub.publish(cmd)

    def _stop(self):
        cmd = Twist()
        self.cmd_pub.publish(cmd)

    @staticmethod
    def _normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _clamp(val, min_val, max_val):
        return max(min_val, min(val, max_val))


def main(args=None):
    rclpy.init(args=args)
    node = GoalSenderNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
