#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""F1TENTH-style Follow-The-Gap (FTG) controller for ROSMASTER R2 (ROS Melodic + Python2.7).

Publishes:
  /cmd_vel_ftg_raw  (geometry_msgs/Twist)

Subscribes:
  /scan  (sensor_msgs/LaserScan)

This node is pure controller logic. A separate node decides when FTG takes control.
"""

import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


class F1TenthFTG(object):
    def __init__(self):
        self.scan_topic = rospy.get_param("~scan_topic", "/scan")
        self.out_topic  = rospy.get_param("~out_topic", "/cmd_vel_ftg_raw")

        # Range handling
        self.max_range_m = float(rospy.get_param("~max_range_m", 6.0))
        self.min_range_m = float(rospy.get_param("~min_range_m", 0.05))

        # Preprocess / smoothing
        self.smoothing_window = int(rospy.get_param("~smoothing_window", 7))  # odd
        self.bubble_radius_m  = float(rospy.get_param("~bubble_radius_m", 0.35))

        # Gap logic
        self.min_gap_width_pts = int(rospy.get_param("~min_gap_width_pts", 8))

        # Steering
        self.steer_gain     = float(rospy.get_param("~steer_gain", 1.2))
        self.max_angular_z  = float(rospy.get_param("~max_angular_z", 1.8))

        # Speed
        self.max_speed       = float(rospy.get_param("~max_speed", 0.7))
        self.min_speed       = float(rospy.get_param("~min_speed", 0.15))
        self.slowdown_dist_m = float(rospy.get_param("~slowdown_dist_m", 1.0))
        self.stop_dist_m     = float(rospy.get_param("~stop_dist_m", 0.35))

        # Forward distance computation
        self.forward_fov_deg = float(rospy.get_param("~forward_fov_deg", 70.0))

        self.pub = rospy.Publisher(self.out_topic, Twist, queue_size=10)
        rospy.Subscriber(self.scan_topic, LaserScan, self.cb_scan, queue_size=1)

        rospy.loginfo("ftg_f1tenth: scan=%s -> %s", self.scan_topic, self.out_topic)

    def smooth(self, arr):
        w = self.smoothing_window
        if w < 3 or (w % 2) == 0 or w >= len(arr):
            return arr
        half = w // 2
        out = list(arr)
        for i in range(half, len(arr) - half):
            s = 0.0
            for k in range(-half, half + 1):
                s += arr[i + k]
            out[i] = s / float(w)
        return out

    def forward_min(self, scan, proc):
        n = len(proc)
        if n < 10:
            return None
        fov = math.radians(self.forward_fov_deg) / 2.0
        inc = max(scan.angle_increment, 1e-6)

        # index of angle 0
        idx0 = int((0.0 - scan.angle_min) / inc)
        span = int(fov / inc)

        a = max(0, idx0 - span)
        b = min(n - 1, idx0 + span)

        m = None
        for r in proc[a:b+1]:
            if r is None:
                continue
            if r > 0.0:
                m = r if m is None else min(m, r)
        return m

    def cb_scan(self, scan):
        ranges = list(scan.ranges)
        n = len(ranges)
        if n < 10:
            return

        # sanitize
        proc = [self.max_range_m] * n
        for i, r in enumerate(ranges):
            if r is None or math.isinf(r) or math.isnan(r):
                proc[i] = self.max_range_m
            else:
                proc[i] = clamp(r, self.min_range_m, self.max_range_m)

        proc = self.smooth(proc)

        # closest point
        closest_i = min(range(n), key=lambda i: proc[i])
        closest_r = proc[closest_i]

        # bubble around closest obstacle
        inc = max(scan.angle_increment, 1e-6)
        bubble_pts = int(self.bubble_radius_m / max(closest_r, 0.10) / max(inc, 1e-3))
        lo = max(0, closest_i - bubble_pts)
        hi = min(n - 1, closest_i + bubble_pts)
        for i in range(lo, hi + 1):
            proc[i] = 0.0

        # find largest gap (contiguous >0)
        best_lo, best_hi = 0, -1
        cur_lo = None
        for i in range(n):
            if proc[i] > 0.0:
                if cur_lo is None:
                    cur_lo = i
            else:
                if cur_lo is not None:
                    cur_hi = i - 1
                    if (cur_hi - cur_lo + 1) > (best_hi - best_lo + 1):
                        best_lo, best_hi = cur_lo, cur_hi
                    cur_lo = None
        if cur_lo is not None:
            cur_hi = n - 1
            if (cur_hi - cur_lo + 1) > (best_hi - best_lo + 1):
                best_lo, best_hi = cur_lo, cur_hi

        cmd = Twist()

        if best_hi < best_lo or (best_hi - best_lo + 1) < self.min_gap_width_pts:
            # no safe gap -> stop
            self.pub.publish(cmd)
            return

        # pick best point in gap: farthest range
        best_i = max(range(best_lo, best_hi + 1), key=lambda i: proc[i])
        angle = scan.angle_min + best_i * scan.angle_increment

        # steering
        cmd.angular.z = clamp(self.steer_gain * angle, -self.max_angular_z, self.max_angular_z)

        # speed based on forward min distance
        fmin = self.forward_min(scan, proc)
        if fmin is None:
            cmd.linear.x = 0.0
        elif fmin <= self.stop_dist_m:
            cmd.linear.x = 0.0
        elif fmin <= self.slowdown_dist_m:
            t = (fmin - self.stop_dist_m) / max(self.slowdown_dist_m - self.stop_dist_m, 1e-3)
            cmd.linear.x = self.min_speed + t * (self.max_speed - self.min_speed)
        else:
            cmd.linear.x = self.max_speed

        self.pub.publish(cmd)


if __name__ == "__main__":
    rospy.init_node("ftg_f1tenth")
    F1TenthFTG()
    rospy.spin()
