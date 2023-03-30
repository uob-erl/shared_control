#!/usr/bin/env python3

import rospy
import sys

from std_msgs.msg import Float32


class Arbitrator:

    def __init__(self):
        rospy.loginfo("INITIALISING NODE -> /adaptive_alpha_node")
        rospy.loginfo("Arbitrator is ACTIVE")
        while not rospy.is_shutdown():
            #Initialize node and log
            
            #Subscribe to entropy
            self.entropy_sub = rospy.Subscriber("/entropy", Float32, self.arbitration_callback)

            self.alpha_pub = rospy.Publisher("/alpha_arb", Float32, queue_size=4)

    def arbitration_callback(self, msg):

        # self.alpha = 0.3
        # self.alpha = 0.5
        # self.alpha = 0.7

        self.alpha = 1-msg.data            #formula
        self.alpha_pub.publish(self.alpha)
 
# def shutdown_log():
#   print("TERMINATING NODE -> /adaptive_alpha_node")
#   print("Arbitrator is NOT ACTIVE")
# rospy.on_shutdown(shutdown_log)

if __name__ == '__main__':
    rospy.init_node("adaptive_alpha_node")
    arb = Arbitrator()
    rospy.spin()




