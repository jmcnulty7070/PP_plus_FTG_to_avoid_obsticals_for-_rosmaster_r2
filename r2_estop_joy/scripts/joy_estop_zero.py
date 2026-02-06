#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class JoyEStopZero(object):
    def __init__(self):
        self.joy_topic = rospy.get_param("~joy_topic", "/joy")
        self.estop_button = int(rospy.get_param("~estop_button", 3))
        self.out_topic = rospy.get_param("~out_topic", "/cmd_vel_safety")
        self.rate_hz = float(rospy.get_param("~rate_hz", 30.0))
        self.hold_mode = bool(rospy.get_param("~hold_mode", True))

        self.estop_pressed = False
        self.pub = rospy.Publisher(self.out_topic, Twist, queue_size=10)
        rospy.Subscriber(self.joy_topic, Joy, self.cb_joy, queue_size=10)

        rospy.loginfo("joy_estop_zero: button=%d -> %s (hold_mode=%s)",
                      self.estop_button, self.out_topic, str(self.hold_mode))

    def cb_joy(self, msg):
        if self.estop_button < len(msg.buttons):
            self.estop_pressed = (msg.buttons[self.estop_button] == 1)
        else:
            self.estop_pressed = False

    def run(self):
        r = rospy.Rate(self.rate_hz)
        z = Twist()
        while not rospy.is_shutdown():
            # publish zero ONLY while button held (so it doesn't fight FTG/auto normally)
            if self.hold_mode and self.estop_pressed:
                self.pub.publish(z)
            r.sleep()

if __name__ == "__main__":
    rospy.init_node("joy_estop_zero", anonymous=False)
    n = JoyEStopZero()
    n.run()

