#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

# Qt via ROS python_qt_binding (works well on ROS Melodic)
from python_qt_binding.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QFrame
from python_qt_binding.QtCore import QTimer


class StatusLight(QWidget):
    """
    Small status box:
      - Green  : PP running
      - Blue   : FTG takeover
      - Yellow : Manual teleop active
      - Gray   : idle/no input
    """

    def __init__(self):
        QWidget.__init__(self)

        # Topics (override with rosparams if you want)
        self.pp_enable_topic = rospy.get_param("~pp_enable_topic", "/pp_enable")
        self.pp_cmd_topic     = rospy.get_param("~pp_cmd_topic", "/cmd_vel_auto")
        self.teleop_topic     = rospy.get_param("~teleop_topic", "/cmd_vel_teleop")
        self.ftg_active_topic = rospy.get_param("~ftg_active_topic", "/ftg_auto_switch/active")

        # Timeouts (seconds)
        self.cmd_timeout_s = float(rospy.get_param("~cmd_timeout_s", 0.5))

        # Internal state
        self.pp_enabled = False
        self.ftg_active = False
        self.last_pp_cmd_t = None
        self.last_teleop_cmd_t = None

        # UI
        self.setWindowTitle("R2 Status Light")
        self.setFixedSize(220, 140)

        layout = QVBoxLayout()

        self.light = QFrame()
        self.light.setFixedSize(140, 60)
        self.light.setStyleSheet("background-color: #777777; border-radius: 10px; border: 2px solid #333333;")
        layout.addWidget(self.light)

        self.label = QLabel("IDLE (no input)")
        self.label.setStyleSheet("font-size: 14px;")
        layout.addWidget(self.label)

        self.setLayout(layout)

        # ROS subscribers
        rospy.Subscriber(self.pp_enable_topic, Bool, self.cb_pp_enable, queue_size=10)
        rospy.Subscriber(self.ftg_active_topic, Bool, self.cb_ftg_active, queue_size=10)
        rospy.Subscriber(self.pp_cmd_topic, Twist, self.cb_pp_cmd, queue_size=10)
        rospy.Subscriber(self.teleop_topic, Twist, self.cb_teleop_cmd, queue_size=10)

        # UI update timer (Qt)
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_ui)
        self.timer.start(100)  # ms

    def now(self):
        return rospy.Time.now().to_sec()

    def recent(self, t):
        if t is None:
            return False
        return (self.now() - t) <= self.cmd_timeout_s

    def cb_pp_enable(self, msg):
        self.pp_enabled = bool(msg.data)

    def cb_ftg_active(self, msg):
        self.ftg_active = bool(msg.data)

    def cb_pp_cmd(self, msg):
        # Any PP cmd seen recently counts as "PP running"
        self.last_pp_cmd_t = self.now()

    def cb_teleop_cmd(self, msg):
        # Only count teleop as active if it is actually commanding something
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.last_teleop_cmd_t = self.now()

    def set_color_text(self, color_hex, text):
        self.light.setStyleSheet("background-color: %s; border-radius: 10px; border: 2px solid #333333;" % color_hex)
        self.label.setText(text)

    def update_ui(self):
        # Determine states
        teleop_active = self.recent(self.last_teleop_cmd_t)
        pp_running = self.pp_enabled and self.recent(self.last_pp_cmd_t)

        # Priority:
        # 1) FTG takeover
        if self.ftg_active:
            self.set_color_text("#2b6cff", "FTG TAKEOVER (safety)")
            return

        # 2) Pure Pursuit running
        if pp_running:
            self.set_color_text("#2ecc71", "PURE PURSUIT RUNNING")
            return

        # 3) Manual teleop active
        if teleop_active:
            self.set_color_text("#f1c40f", "MANUAL DRIVE (teleop)")
            return

        # 4) Idle
        self.set_color_text("#777777", "IDLE (no input)")


def main():
    rospy.init_node("r2_status_light", anonymous=False)

    app = QApplication([])
    w = StatusLight()
    w.show()

    # Keep Qt alive while ROS runs
    app.exec_()


if __name__ == "__main__":
    main()
