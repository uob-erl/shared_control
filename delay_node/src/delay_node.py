#!/usr/bin/env python3

import functools
import rospy
import sys

from geometry_msgs.msg import Twist


class delayer:

    def __init__(self):
        self.sub = rospy.Subscriber("/teleop/cmd_vel", Twist, self.callback)
        self.pub = rospy.Publisher("/delayed_teleop/cmd_vel", Twist, queue_size=4)

        self.lag = 0.05 # in seconds

    def delayed_callback(self, msg, event):
        self.pub.publish(msg)

    def callback(self, msg):
        timer = rospy.Timer(rospy.Duration(self.lag), functools.partial(
            self.delayed_callback, msg), oneshot=True)

if __name__ == '__main__':
    rospy.init_node("delay_node")
    dela = delayer()
    rospy.spin()

