#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""Automatic FTG takeover (or blend) node for ROSMASTER R2.

LOCKED interfaces preserved from your working zip:
  - /pp_enable (Bool) deadman enable (mode_hold.py)
  - /cmd_vel_auto_raw -> /cmd_vel_auto (cmdvel_gate.py)
  - twist_mux priorities: safety > teleop > auto
  - /cmd_vel_safety is the highest-priority safety channel into twist_mux

Inputs:
  /cmd_vel_auto        (Pure Pursuit after gate)
  /cmd_vel_ftg_raw     (FTG controller output)
  /scan                (LaserScan)
  /pp_enable           (Bool)  - your autonomy deadman topic

Output:
  /cmd_vel_safety      (Twist)

Modes:
  - switch: FTG fully replaces PP when obstacle is close
  - blend:  FTG blends angular + reduces linear while risky

Set ~mode: 'switch' or 'blend'
"""

import math
import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def is_finite(x):
    """Python2.7-safe finiteness check for floats."""
    if x is None:
        return False
    try:
        # math.isnan / math.isinf exist in Python 2.7
        return (not math.isnan(x)) and (not math.isinf(x))
    except Exception:
        return False


class FtgAutoSwitch(object):
    def __init__(self):
        self.mode = rospy.get_param("~mode", "switch")  # 'switch' | 'blend'

        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.enable_topic = rospy.get_param("~enable_topic", "/pp_enable")

        self.pp_topic = rospy.get_param("~pp_cmd_topic", "/cmd_vel_auto")
        self.ftg_topic = rospy.get_param("~ftg_cmd_topic", "/cmd_vel_ftg_raw")
        self.out_topic = rospy.get_param("~out_cmd_topic", "/cmd_vel_safety")

        # takeover thresholds (hysteresis)
        self.switch_dist_m = float(rospy.get_param("~switch_dist_m", 0.90))
        self.release_dist_m = float(rospy.get_param("~release_dist_m", 1.20))

        # forward distance computation
        self.front_fov_deg = float(rospy.get_param("~front_fov_deg", 70.0))

        # blend parameters
        self.stop_dist_m = float(rospy.get_param("~stop_dist_m", 0.35))
        self.min_speed_scale = float(rospy.get_param("~min_speed_scale", 0.35))
        self.max_steer_rad_s = float(rospy.get_param("~max_steer_rad_s", 1.5))  # match your current safety clamp

        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))

        self.enabled = False
        self.active = False

        self.pp_cmd = Twist()
        self.ftg_cmd = Twist()
        self.front_min = None

        self.pub = rospy.Publisher(self.out_topic, Twist, queue_size=10)

        rospy.Subscriber(self.enable_topic, Bool, self.cb_enable, queue_size=10)
        rospy.Subscriber(self.pp_topic, Twist, self.cb_pp, queue_size=10)
        rospy.Subscriber(self.ftg_topic, Twist, self.cb_ftg, queue_size=10)
        rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=1)

        rospy.loginfo("ftg_auto_switch: mode=%s enable=%s scan=%s", self.mode, self.enable_topic, self.scan_topic)
        rospy.loginfo("ftg_auto_switch: pp=%s ftg=%s -> %s", self.pp_topic, self.ftg_topic, self.out_topic)

    def cb_enable(self, msg):
        self.enabled = bool(msg.data)

    def cb_pp(self, msg):
        self.pp_cmd = msg

    def cb_ftg(self, msg):
        self.ftg_cmd = msg

    def cb_scan(self, scan):
        # Defensive: scan could be empty during startup
        n = len(scan.ranges)
        if n < 5:
            self.front_min = None
            self.active = True
            return

        # Compute indices covering a forward field of view around angle=0
        fov = math.radians(self.front_fov_deg) / 2.0
        inc = max(scan.angle_increment, 1e-6)

        def idx(angle):
            return int((angle - scan.angle_min) / inc)

        i0 = idx(-fov)
        i1 = idx(fov)

        # Clamp bounds
        i0 = max(0, min(n - 1, i0))
        i1 = max(0, min(n - 1, i1))
        if i1 < i0:
            i0, i1 = i1, i0

        # Find minimum valid range in that forward window
        m = None
        for r in scan.ranges[i0:i1 + 1]:
            if not is_finite(r):
                continue
            if r <= 0.01:
                continue
            m = r if m is None else min(m, r)

        self.front_min = m

        # hysteresis: switch on when too close, release when clear
        if self.front_min is None:
            # If scan is invalid, be conservative: treat as active safety
            self.active = True
            return

        if (not self.active) and (self.front_min < self.switch_dist_m):
            self.active = True
        elif self.active and (self.front_min > self.release_dist_m):
            self.active = False

    def blend_alpha(self, d):
        # 0 when far, 1 when very close
        if d is None:
            return 1.0
        if d <= self.stop_dist_m:
            return 1.0
        if d >= self.switch_dist_m:
            return 0.0
        t = (self.switch_dist_m - d) / max(self.switch_dist_m - self.stop_dist_m, 1e-3)
        return clamp(t, 0.0, 1.0)

    def speed_scale(self, d):
        # 1 when far, down to min_speed_scale near switch, to 0 at stop
        if d is None:
            return 0.0
        if d <= self.stop_dist_m:
            return 0.0
        if d >= self.switch_dist_m:
            return 1.0
        t = (d - self.stop_dist_m) / max(self.switch_dist_m - self.stop_dist_m, 1e-3)
        return clamp(self.min_speed_scale + t * (1.0 - self.min_speed_scale), 0.0, 1.0)

    def run(self):
        r = rospy.Rate(self.rate_hz)
        zero = Twist()

        while not rospy.is_shutdown():
            if not self.enabled:
                # deadman not held -> publish zero safety
                self.pub.publish(zero)
                r.sleep()
                continue

            if self.mode == "switch":
                out = self.ftg_cmd if self.active else self.pp_cmd
            else:
                if self.active:
                    a = self.blend_alpha(self.front_min)
                    s = self.speed_scale(self.front_min)
                else:
                    a = 0.0
                    s = 1.0

                out = Twist()
                out.linear.x = s * ((1.0 - a) * self.pp_cmd.linear.x + a * self.ftg_cmd.linear.x)
                out.angular.z = (1.0 - a) * self.pp_cmd.angular.z + a * self.ftg_cmd.angular.z

            out.angular.z = clamp(out.angular.z, -self.max_steer_rad_s, self.max_steer_rad_s)
            self.pub.publish(out)
            r.sleep()


if __name__ == "__main__":
    rospy.init_node("ftg_auto_switch", anonymous=False)
    node = FtgAutoSwitch()
    node.run()

