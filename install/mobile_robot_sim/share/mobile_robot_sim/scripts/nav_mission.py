#!/usr/bin/env python3
"""
Navigation Mission – Smooth mecanum holonomic navigation
----------------------------------------------------------
Robot spawns at world (0.93, -1.67) with yaw=1.5708 → odom (0,0).
Odom frame: odom_x = world_y, odom_y = -world_x (relative to spawn).

Pure translational movement — the robot never rotates intentionally.
A small angular.z correction keeps the heading locked to the initial
yaw.  Velocity commands are smoothed with a low-pass filter for
clean, straight paths.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool


class NavMission(Node):

    MAX_SPEED    = 3.0
    TIMER_PERIOD = 0.1       # 10 Hz

    GOAL_TOL     = 0.05      # m – tight centering
    SLOW_RADIUS  = 0.5       # m – begin decelerating

    OBS_HARD     = 0.30      # m
    OBS_SOFT     = 0.9       # m
    SMOOTH       = 0.3       # low-pass filter: 0=instant, 1=no change
    YAW_KP       = 2.0       # yaw correction gain

    LID_OPEN_TICKS  = 200    # 20 s lid open at 10 Hz
    LID_CLOSE_TICKS = 30     # 3 s for lid to close

    # Spawn world (0.93, -1.67)
    # marker_wp3: world (2.0, 0.5) → odom (2.17, -1.07)
    WAYPOINTS = [
        (2.17, -1.07),    # marker_wp3
    ]

    def __init__(self):
        super().__init__('nav_mission')

        self.px = 0.0
        self.py = 0.0
        self.yaw = 0.0
        self._init_yaw = None     # locked at first odom reading
        self._got_odom = False

        self.rf = float('inf')
        self.rb = float('inf')
        self.rl = float('inf')
        self.rr = float('inf')

        self._wi = 0
        self._done = False
        self._dwell = 0
        self._dwell_phase = None   # None | 'open' | 'closing'
        self._logc = 0
        self._slide_dir = 0.0

        # Smoothed velocity state
        self._vx = 0.0
        self._vy = 0.0

        self.create_subscription(Odometry,  '/odom',             self._odom_cb, 10)
        self.create_subscription(LaserScan, '/ultrasonic/front', self._f_cb, 10)
        self.create_subscription(LaserScan, '/ultrasonic/back',  self._b_cb, 10)
        self.create_subscription(LaserScan, '/ultrasonic/left',  self._l_cb, 10)
        self.create_subscription(LaserScan, '/ultrasonic/right', self._r_cb, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cli = self.create_client(SetBool, '/robot_enable')
        self.lid_cli = self.create_client(SetBool, '/lid_control')
        self._enable()

        self.create_timer(self.TIMER_PERIOD, self._loop)
        wp = self.WAYPOINTS[0]
        self.get_logger().info(
            f'Mission started – first target: odom({wp[0]:.2f},{wp[1]:.2f})')

    def _enable(self):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            return
        req = SetBool.Request(); req.data = True
        self.cli.call_async(req)

    def _lid(self, open_it: bool):
        if not self.lid_cli.service_is_ready():
            return
        req = SetBool.Request(); req.data = open_it
        self.lid_cli.call_async(req)

    # ---- sensor callbacks -------------------------------------------- #
    def _odom_cb(self, m):
        self.px = m.pose.pose.position.x
        self.py = m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                              1 - 2*(q.y*q.y + q.z*q.z))
        if self._init_yaw is None:
            self._init_yaw = self.yaw
        self._got_odom = True

    @staticmethod
    def _mr(m):
        v = [r for r in m.ranges if m.range_min <= r <= m.range_max]
        return min(v) if v else float('inf')

    def _f_cb(self, m): self.rf = self._mr(m)
    def _b_cb(self, m): self.rb = self._mr(m)
    def _l_cb(self, m): self.rl = self._mr(m)
    def _r_cb(self, m): self.rr = self._mr(m)

    # ---- helpers ----------------------------------------------------- #
    def _wp(self):
        return self.WAYPOINTS[self._wi]

    def _dist(self):
        w = self._wp()
        return math.hypot(w[0] - self.px, w[1] - self.py)

    def _rep(self, d):
        if d >= self.OBS_SOFT: return 0.0
        if d <= self.OBS_HARD: return 1.0
        return (self.OBS_SOFT - d) / (self.OBS_SOFT - self.OBS_HARD)

    def _odom_to_body(self, ox, oy):
        """Use INITIAL yaw so small drift doesn't wobble the path."""
        c = math.cos(self._init_yaw)
        s = math.sin(self._init_yaw)
        bx =  c * ox + s * oy
        by = -s * ox + c * oy
        return bx, by

    @staticmethod
    def _wrap(a):
        while a >  math.pi: a -= 2*math.pi
        while a < -math.pi: a += 2*math.pi
        return a

    # ================================================================== #
    def _loop(self):
        if self._done or not self._got_odom:
            return

        dist = self._dist()

        # --- periodic log ---
        self._logc += 1
        if self._logc % 10 == 0:
            w = self._wp()
            self.get_logger().info(
                f'WP{self._wi+1}/{len(self.WAYPOINTS)} '
                f'tgt=({w[0]:.2f},{w[1]:.2f}) '
                f'pos=({self.px:.2f},{self.py:.2f}) '
                f'd={dist:.2f} yaw={math.degrees(self.yaw):.0f}° '
                f'F:{self.rf:.2f} B:{self.rb:.2f} '
                f'L:{self.rl:.2f} R:{self.rr:.2f}')

        # --- dwell (lid open → wait → lid close → advance) ---
        if self._dwell > 0:
            self._dwell -= 1
            self.pub.publish(Twist())
            self._vx = self._vy = 0.0

            if self._dwell_phase == 'open' and self._dwell == 0:
                # Lid has been open 20 s → close it
                self._lid(False)
                self.get_logger().info('  Lid closing...')
                self._dwell_phase = 'closing'
                self._dwell = self.LID_CLOSE_TICKS
            elif self._dwell_phase == 'closing' and self._dwell == 0:
                # Lid closed → advance to next waypoint
                self._wi += 1
                self._dwell_phase = None
                if self._wi >= len(self.WAYPOINTS):
                    self.get_logger().info('=== MISSION COMPLETE ===')
                    self._done = True
                    return
                nw = self.WAYPOINTS[self._wi]
                self.get_logger().info(
                    f'  Next WP{self._wi+1} odom({nw[0]:.2f},{nw[1]:.2f})')
            return

        # --- reached waypoint? ---
        if dist < self.GOAL_TOL:
            w = self._wp()
            self.get_logger().info(
                f'>>> REACHED WP{self._wi+1} odom({w[0]:.2f},{w[1]:.2f})  '
                f'err={dist:.3f}m')
            self.pub.publish(Twist())
            self._vx = self._vy = 0.0
            self._lid(True)
            self.get_logger().info('  Lid opening – staying 20 s...')
            self._dwell_phase = 'open'
            self._dwell = self.LID_OPEN_TICKS
            return

        # ============================================================ #
        #  SMOOTH DIRECT VECTOR + SIDEWAYS SLIDE                        #
        # ============================================================ #
        w = self._wp()
        dx = w[0] - self.px
        dy = w[1] - self.py
        mag = math.hypot(dx, dy)

        speed = self.MAX_SPEED
        if dist < self.SLOW_RADIUS:
            speed *= max(0.25, dist / self.SLOW_RADIUS)

        # Direct vector toward target (odom frame → body frame)
        if mag > 0.001:
            ox = (dx / mag) * speed
            oy = (dy / mag) * speed
        else:
            ox = oy = 0.0
        gvx, gvy = self._odom_to_body(ox, oy)

        # --- Obstacle slide (body frame) ---
        rep_f = self._rep(self.rf)
        rep_l = self._rep(self.rl)
        rep_r = self._rep(self.rr)

        if dist < 0.4:
            fade = max(0.05, dist / 0.4)
            rep_f *= fade; rep_l *= fade; rep_r *= fade

        obstacle_ahead = rep_f > 0.1

        if obstacle_ahead and self._slide_dir == 0.0:
            self._slide_dir = 1.0 if self.rr > self.rl else -1.0
        elif not obstacle_ahead:
            self._slide_dir = 0.0

        if obstacle_ahead:
            gvx *= (1.0 - rep_f)
            gvy += self._slide_dir * speed * rep_f * 0.5

        if rep_l > 0.1:
            gvy -= speed * rep_l * 0.2
        if rep_r > 0.1:
            gvy += speed * rep_r * 0.2

        # Clamp
        spd = math.hypot(gvx, gvy)
        if spd > self.MAX_SPEED:
            sc = self.MAX_SPEED / spd
            gvx *= sc; gvy *= sc

        # --- Low-pass filter for smooth acceleration ---
        a = self.SMOOTH
        self._vx = a * self._vx + (1 - a) * gvx
        self._vy = a * self._vy + (1 - a) * gvy

        # --- Yaw correction: keep heading locked ---
        yaw_err = self._wrap(self._init_yaw - self.yaw)
        wz = self.YAW_KP * yaw_err

        tw = Twist()
        tw.linear.x = self._vx
        tw.linear.y = self._vy
        tw.angular.z = wz
        self.pub.publish(tw)


def main(args=None):
    rclpy.init(args=args)
    node = NavMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.pub.publish(Twist())
        node.get_logger().info('Stop')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
