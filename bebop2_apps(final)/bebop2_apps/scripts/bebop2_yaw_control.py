#!/usr/bin/env python

import rospy
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged
from geometry_msgs.msg import Twist
from math import pi

class YawControl:
    def __init__(self):
        rospy.init_node("yaw_control", anonymous=True)
        self.sub = rospy.Subscriber("/bebop/states/ardrone3/PilotingState/FlyingStateChanged", Ardrone3PilotingStateAttitudeChanged, self.set_yaw, queue_size=1)
        self.pub = rospy.Publisher("bebop/cmd_vel", Twist, queue_size=1)
        self.twist = Twist()
        self.twist.angular.z = 1.0
        self.rate = rospy.Rate(10)

    def set_yaw(self, msg):
        # TODO: 
        goto_yaw = float(input("enter yaw: "))

        while msg.yaw 

        self.state_attitude.yaw = goto_yaw
        self.pub.publish(self.twist)

if __name__=="__main__":
    try:
        ctrl = YawControl()
        ctrl.set_yaw()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass