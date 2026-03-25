#!/usr/bin/env python3
"""
Obstacle Navigator Node
-----------------------
Drives the mecanum robot forward while using the 4 ultrasonic sensors
(front, back, left, right) to detect static obstacles (tables, shelves, etc.)
and the IMU to track heading.  When an obstacle is detected within the
safety threshold the robot stops, decides on an evasive direction based on
which sensors are clear, strafes or rotates around the obstacle, then
resumes forward motion.

Sensors (LaserScan, min 0.12 m, max 10 m, 5 rays ±15°):
    /ultrasonic/front   – forward-facing
    /ultrasonic/back    – rear-facing
    /ultrasonic/left    – left-facing
    /ultrasonic/right   – right-facing

IMU (sensor_msgs/Imu):
    /imu                – orientation & angular velocity

Actuator:
    /cmd_vel            – geometry_msgs/Twist  (mecanum holonomic)

Service:
    /robot_enable       – std_srvs/SetBool  (must be called True first)
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu
from geometry_msgs.msg import Twist
from std_srvs.srv import SetBool


class ObstacleNavigator(Node):

    # ---------- tunables ----------
    OBSTACLE_DIST = 0.50       # metres – stop if anything closer
    SLOW_DIST = 0.90           # metres – reduce speed in this zone
    FORWARD_SPEED = 0.8        # m/s cruise
    STRAFE_SPEED = 0.6         # m/s lateral dodge
    ROTATE_SPEED = 1.2         # rad/s turning
    TIMER_PERIOD = 0.1         # 10 Hz control loop

    def __init__(self):
        super().__init__('obstacle_navigator')

        # --- latest range per direction (initialise to "far away") ---
        self.range_front = float('inf')
        self.range_back = float('inf')
        self.range_left = float('inf')
        self.range_right = float('inf')

        # --- IMU state ---
        self.yaw = 0.0          # radians, integrated from gyro_z
        self.last_imu_time = None

        # --- subscribers ---
        self.create_subscription(LaserScan, '/ultrasonic/front',
                                 self._cb_front, 10)
        self.create_subscription(LaserScan, '/ultrasonic/back',
                                 self._cb_back, 10)
        self.create_subscription(LaserScan, '/ultrasonic/left',
                                 self._cb_left, 10)
        self.create_subscription(LaserScan, '/ultrasonic/right',
                                 self._cb_right, 10)
        self.create_subscription(Imu, '/imu', self._cb_imu, 10)

        # --- publisher ---
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- enable the robot via service ---
        self.enable_client = self.create_client(SetBool, '/robot_enable')
        self._enable_robot()

        # --- control loop ---
        self.create_timer(self.TIMER_PERIOD, self._control_loop)

        self.get_logger().info('Obstacle Navigator started – waiting for sensor data …')

    # ------------------------------------------------------------------ #
    #  Service helper                                                     #
    # ------------------------------------------------------------------ #
    def _enable_robot(self):
        """Call /robot_enable True so the start_stop_node lets motion through."""
        if not self.enable_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn('/robot_enable service not available – continuing anyway')
            return
        req = SetBool.Request()
        req.data = True
        future = self.enable_client.call_async(req)
        future.add_done_callback(
            lambda f: self.get_logger().info('Robot enabled ✓')
            if f.result() is not None else None)

    # ------------------------------------------------------------------ #
    #  Sensor callbacks                                                   #
    # ------------------------------------------------------------------ #
    @staticmethod
    def _min_range(msg: LaserScan) -> float:
        """Return the minimum valid range from a LaserScan message."""
        valid = [r for r in msg.ranges
                 if msg.range_min <= r <= msg.range_max]
        return min(valid) if valid else float('inf')

    def _cb_front(self, msg):
        self.range_front = self._min_range(msg)

    def _cb_back(self, msg):
        self.range_back = self._min_range(msg)

    def _cb_left(self, msg):
        self.range_left = self._min_range(msg)

    def _cb_right(self, msg):
        self.range_right = self._min_range(msg)

    def _cb_imu(self, msg: Imu):
        """Integrate gyroscope Z to track yaw heading."""
        now = self.get_clock().now()
        if self.last_imu_time is not None:
            dt = (now - self.last_imu_time).nanoseconds * 1e-9
            self.yaw += msg.angular_velocity.z * dt
            # keep in [-π, π]
            self.yaw = math.atan2(math.sin(self.yaw), math.cos(self.yaw))
        self.last_imu_time = now

    # ------------------------------------------------------------------ #
    #  Main control loop                                                  #
    # ------------------------------------------------------------------ #
    def _control_loop(self):
        twist = Twist()

        f = self.range_front
        b = self.range_back
        l = self.range_left
        r = self.range_right

        front_blocked = f < self.OBSTACLE_DIST
        left_blocked = l < self.OBSTACLE_DIST
        right_blocked = r < self.OBSTACLE_DIST

        front_slow = f < self.SLOW_DIST

        # Log current distances every ~1 s (every 10th tick)
        if hasattr(self, '_log_cnt'):
            self._log_cnt += 1
        else:
            self._log_cnt = 0
        if self._log_cnt % 10 == 0:
            self.get_logger().info(
                f'Ranges  F:{f:.2f}  B:{b:.2f}  L:{l:.2f}  R:{r:.2f}  '
                f'Yaw:{math.degrees(self.yaw):.1f}°')

        # ---- decision tree ----

        if front_blocked and left_blocked and right_blocked:
            # boxed in on three sides → reverse
            twist.linear.x = -self.FORWARD_SPEED
            self.get_logger().info('REVERSE – boxed in')

        elif front_blocked:
            # obstacle ahead → pick a free side to strafe
            if not right_blocked and (right_blocked == left_blocked or r > l):
                # strafe right (positive y in our frame is right)
                twist.linear.y = self.STRAFE_SPEED
                self.get_logger().info(
                    f'STRAFE RIGHT – front obstacle at {f:.2f} m')
            elif not left_blocked:
                # strafe left
                twist.linear.y = -self.STRAFE_SPEED
                self.get_logger().info(
                    f'STRAFE LEFT – front obstacle at {f:.2f} m')
            else:
                # both sides blocked → rotate in place
                twist.angular.z = self.ROTATE_SPEED
                self.get_logger().info(
                    f'ROTATE – front obstacle at {f:.2f} m, sides blocked')

        elif left_blocked and not right_blocked:
            # too close on the left → nudge right while moving forward
            twist.linear.x = self.FORWARD_SPEED * 0.5
            twist.linear.y = self.STRAFE_SPEED * 0.5
            self.get_logger().info(
                f'NUDGE RIGHT – left obstacle at {l:.2f} m')

        elif right_blocked and not left_blocked:
            # too close on the right → nudge left while moving forward
            twist.linear.x = self.FORWARD_SPEED * 0.5
            twist.linear.y = -self.STRAFE_SPEED * 0.5
            self.get_logger().info(
                f'NUDGE LEFT – right obstacle at {r:.2f} m')

        elif front_slow:
            # approaching something – slow down
            twist.linear.x = self.FORWARD_SPEED * 0.4
            self.get_logger().info(
                f'SLOW – obstacle ahead at {f:.2f} m')

        else:
            # all clear → cruise forward
            twist.linear.x = self.FORWARD_SPEED

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNavigator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        # stop the robot before shutting down
        node.cmd_pub.publish(Twist())
        node.get_logger().info('Stopping robot …')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
