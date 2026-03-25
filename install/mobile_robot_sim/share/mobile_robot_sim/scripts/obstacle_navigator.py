#!/usr/bin/env python3
"""
Obstacle Navigator – True Mecanum Holonomic Navigation
------------------------------------------------------
Empirically calibrated.  The MecanumDrive odom frame starts at (0,0,yaw=0)
aligned with the robot body: odom +x = forward (world +Y at spawn),
odom +y = left (world -X at spawn).

Waypoints are pre-computed in the odom frame:
  odom_x = world_y,  odom_y = -world_x

The robot drives toward each waypoint using pure body-frame vx/vy
commands — no rotation needed.  Obstacle avoidance adds small
repulsive body-frame velocities from the four ultrasonic sensors.

Route: marker 2 → marker 3 → marker 1 → start (brown).
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import SetBool


class ObstacleNavigator(Node):

    # Commanded speed (actual speed ≈ 15-20% of this due to physics)
    MAX_SPEED    = 3.0
    TIMER_PERIOD = 0.05     # 20 Hz

    GOAL_TOL     = 0.15     # m – centered on the 0.4m-radius circle
    SLOW_RADIUS  = 0.6      # m – begin decelerating
    AXIS_TOL     = 0.20     # m – switch to next axis when primary < this

    OBS_HARD     = 0.30     # m – full stop
    OBS_SOFT     = 0.80     # m – begin repulsion
    REP_GAIN     = 1.5      # repulsion strength (body frame)

    DWELL_TICKS  = 60       # 3 s at 20 Hz

    STUCK_WINDOW  = 300     # 15 s
    STUCK_THRESH  = 260
    REROUTE_TICKS = 60

    # Waypoints in ODOM frame (pre-computed from world coords)
    # World→Odom transform: odom_x = world_y, odom_y = -world_x
    # (robot spawns at world origin facing +Y, odom starts at (0,0,yaw=0))
    WAYPOINTS = [
        ( 1.5,   0.0),   # marker 2: world ( 0.0,  1.5) – straight ahead
        ( 0.5,  -2.0),   # marker 3: world ( 2.0,  0.5)
        (-1.0,   2.0),   # marker 1: world (-2.0, -1.0)
        ( 0.0,   0.0),   # start:    world ( 0.0,  0.0)
    ]

    def __init__(self):
        super().__init__('obstacle_navigator')

        self.px = 0.0
        self.py = 0.0
        self.yaw = 0.0
        self._got_odom = False

        self.rf = float('inf')
        self.rb = float('inf')
        self.rl = float('inf')
        self.rr = float('inf')

        self._wi = 0
        self._done = False
        self._dwell = 0
        self._hist = []
        self._reroute = 0
        self._rdir = 1.0
        self._logc = 0

        self.create_subscription(Odometry,  '/odom',             self._odom_cb, 10)
        self.create_subscription(LaserScan, '/ultrasonic/front', self._f_cb, 10)
        self.create_subscription(LaserScan, '/ultrasonic/back',  self._b_cb, 10)
        self.create_subscription(LaserScan, '/ultrasonic/left',  self._l_cb, 10)
        self.create_subscription(LaserScan, '/ultrasonic/right', self._r_cb, 10)

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.cli = self.create_client(SetBool, '/robot_enable')
        self._enable()

        self.create_timer(self.TIMER_PERIOD, self._loop)
        wp = self.WAYPOINTS[0]
        self.get_logger().info(
            f'Nav started – WP1 odom({wp[0]:.1f},{wp[1]:.1f})')

    def _enable(self):
        if not self.cli.wait_for_service(timeout_sec=5.0):
            return
        req = SetBool.Request(); req.data = True
        self.cli.call_async(req)

    # ---- sensor callbacks -------------------------------------------- #
    def _odom_cb(self, m):
        self.px = m.pose.pose.position.x
        self.py = m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.yaw = math.atan2(2*(q.w*q.z + q.x*q.y),
                              1 - 2*(q.y*q.y + q.z*q.z))
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
        """Transform odom-frame vector into robot body frame."""
        c = math.cos(self.yaw)
        s = math.sin(self.yaw)
        bx =  c * ox + s * oy
        by = -s * ox + c * oy
        return bx, by

    def _record(self, ev):
        self._hist.append(ev)
        if len(self._hist) > self.STUCK_WINDOW:
            self._hist.pop(0)

    def _stuck(self):
        return (len(self._hist) >= self.STUCK_WINDOW and
                sum(self._hist) >= self.STUCK_THRESH)

    # ================================================================== #
    def _loop(self):
        if self._done or not self._got_odom:
            return

        dist = self._dist()

        # --- periodic log ---
        self._logc += 1
        if self._logc % 20 == 0:
            w = self._wp()
            self.get_logger().info(
                f'WP{self._wi+1}/{len(self.WAYPOINTS)} '
                f'tgt=({w[0]:.2f},{w[1]:.2f}) '
                f'pos=({self.px:.2f},{self.py:.2f}) '
                f'd={dist:.2f} yaw={math.degrees(self.yaw):.0f}° '
                f'F:{self.rf:.2f} B:{self.rb:.2f} '
                f'L:{self.rl:.2f} R:{self.rr:.2f}')

        # --- dwell (parked on circle) ---
        if self._dwell > 0:
            self._dwell -= 1
            self.pub.publish(Twist())
            if self._dwell == 0:
                self._wi += 1
                self._hist.clear()
                self._reroute = 0
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
            self._dwell = self.DWELL_TICKS
            return

        # --- reroute recovery ---
        if self._reroute > 0:
            self._reroute -= 1
            tw = Twist()
            p = self._reroute
            d = self._rdir
            if p > 55:
                tw.linear.x = -1.5
            elif p > 25:
                tw.linear.y = d * 2.0
            else:
                tw.linear.x = 2.0
            self.pub.publish(tw)
            if self._reroute == 0:
                self._hist.clear()
            return

        # --- stuck? ---
        if self._stuck():
            self._rdir = 1.0 if self.rr > self.rl else -1.0
            self.get_logger().info('STUCK – rerouting')
            self._reroute = self.REROUTE_TICKS
            self._hist.clear()
            return

        # ============================================================ #
        #  SEQUENTIAL-AXIS NAVIGATION (one axis at a time)              #
        # ============================================================ #

        w = self._wp()
        dx = w[0] - self.px   # odom-frame X error
        dy = w[1] - self.py   # odom-frame Y error

        # Speed ramps down near goal for precise centering
        speed = self.MAX_SPEED
        if dist < self.SLOW_RADIUS:
            speed *= max(0.15, dist / self.SLOW_RADIUS)

        # Move one axis at a time: correct the bigger axis first,
        # then the smaller axis, then fine direct approach.
        if abs(dx) >= self.AXIS_TOL and abs(dx) >= abs(dy):
            # Phase A: drive forward/backward only
            ox_goal = speed if dx > 0 else -speed
            oy_goal = 0.0
        elif abs(dy) >= self.AXIS_TOL:
            # Phase B: strafe left/right only
            ox_goal = 0.0
            oy_goal = speed if dy > 0 else -speed
        elif abs(dx) >= self.AXIS_TOL:
            # Remaining X after Y done
            ox_goal = speed if dx > 0 else -speed
            oy_goal = 0.0
        else:
            # Fine approach: direct vector for last few cm
            mag = math.hypot(dx, dy)
            if mag > 0.001:
                ox_goal = (dx / mag) * speed
                oy_goal = (dy / mag) * speed
            else:
                ox_goal = oy_goal = 0.0

        # Transform odom-frame command to body frame
        gvx, gvy = self._odom_to_body(ox_goal, oy_goal)

        # 2) Obstacle avoidance (body frame)
        rep_f = self._rep(self.rf)
        rep_b = self._rep(self.rb)
        rep_l = self._rep(self.rl)
        rep_r = self._rep(self.rr)

        # Fade repulsion near goal so robot commits to the circle
        if dist < 0.4:
            fade = max(0.05, dist / 0.4)
            rep_f *= fade; rep_b *= fade
            rep_l *= fade; rep_r *= fade

        # Hard stop: if moving toward an obstacle that is very close,
        # zero out the goal velocity on that axis
        if rep_f > 0.7 and gvx > 0:    # obstacle ahead & driving forward
            gvx = 0.0
        if rep_b > 0.7 and gvx < 0:    # obstacle behind & reversing
            gvx = 0.0
        if rep_l > 0.7 and gvy > 0:    # obstacle left & strafing left
            gvy = 0.0
        if rep_r > 0.7 and gvy < 0:    # obstacle right & strafing right
            gvy = 0.0

        rvx = self.REP_GAIN * (-rep_f + rep_b)
        rvy = self.REP_GAIN * (-rep_l + rep_r)

        # Dodge sideways when obstacle ahead
        if rep_f > 0.3:
            side = 1.0 if self.rr > self.rl else -1.0
            rvy += self.REP_GAIN * rep_f * 0.6 * side
        if rep_b > 0.3:
            side = 1.0 if self.rr > self.rl else -1.0
            rvy += self.REP_GAIN * rep_b * 0.4 * side

        # 3) Combine
        vx = gvx + rvx
        vy = gvy + rvy

        # Clamp
        spd = math.hypot(vx, vy)
        if spd > self.MAX_SPEED:
            sc = self.MAX_SPEED / spd
            vx *= sc; vy *= sc

        tw = Twist()
        tw.linear.x = vx
        tw.linear.y = vy
        self.pub.publish(tw)

        # Only count as evasive when obstacle repulsion is dominant
        self._record(max(rep_f, rep_l, rep_r) > 0.5)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleNavigator()
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
