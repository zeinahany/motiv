#!/usr/bin/env python3
"""Quick diagnostic: send vx-only then vy-only and record odom delta."""
import rclpy, time, math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool

class AxisTest(Node):
    def __init__(self):
        super().__init__('axis_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ox = self.oy = self.oyaw = None
        self.create_subscription(Odometry, '/odom', self._cb, 10)
        self.cli = self.create_client(SetBool, '/robot_enable')

    def enable(self):
        if self.cli.wait_for_service(timeout_sec=5.0):
            req = SetBool.Request(); req.data = True
            fut = self.cli.call_async(req)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
            print("Robot enabled")
        else:
            print("WARNING: /robot_enable not available")

    def _cb(self, m):
        self.ox = m.pose.pose.position.x
        self.oy = m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.oyaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))

    def wait_odom(self):
        while self.ox is None:
            rclpy.spin_once(self, timeout_sec=0.1)

    def send(self, vx, vy, dur):
        tw = Twist(); tw.linear.x = vx; tw.linear.y = vy
        t0 = time.time()
        while time.time() - t0 < dur:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
        self.pub.publish(Twist())  # stop
        rclpy.spin_once(self, timeout_sec=0.2)

def main():
    rclpy.init()
    n = AxisTest()
    n.wait_odom()
    n.enable()
    time.sleep(0.5)
    print(f"Initial odom: x={n.ox:.4f}  y={n.oy:.4f}  yaw={math.degrees(n.oyaw):.1f}°")

    # Test 1: vx only (5s at 1.0)
    x0, y0 = n.ox, n.oy
    n.send(1.0, 0.0, 5.0)
    dx, dy = n.ox - x0, n.oy - y0
    print(f"After vx=1.0 for 5s:  dx={dx:.4f}  dy={dy:.4f}  yaw={math.degrees(n.oyaw):.1f}°")

    time.sleep(1.0)
    rclpy.spin_once(n, timeout_sec=0.2)

    # Test 2: vy only (5s at 1.0)
    x0, y0 = n.ox, n.oy
    n.send(0.0, 1.0, 5.0)
    dx, dy = n.ox - x0, n.oy - y0
    print(f"After vy=1.0 for 5s:  dx={dx:.4f}  dy={dy:.4f}  yaw={math.degrees(n.oyaw):.1f}°")

    time.sleep(1.0)
    rclpy.spin_once(n, timeout_sec=0.2)

    # Test 3: negative vx
    x0, y0 = n.ox, n.oy
    n.send(-1.0, 0.0, 3.0)
    dx, dy = n.ox - x0, n.oy - y0
    print(f"After vx=-1.0 for 3s: dx={dx:.4f}  dy={dy:.4f}  yaw={math.degrees(n.oyaw):.1f}°")

    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
