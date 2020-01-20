#!/usr/bin/env python

import rospy
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged

class SubAtti:

    def __init__(self):
        rospy.init_node('get_attitude', anonymous = True)
                                     
        rospy.Subscriber('/bebop/states/ardrone3/PilotingState/AttitudeChanged',
                         Ardrone3PilotingStateAttitudeChanged,
                         self.get_atti, queue_size = 1)
                                     
        self.atti = Ardrone3PilotingStateAttitudeChanged()
              
    def get_atti(self, msg):
        self.atti = msg
        
        print("---")
        print("yaw = %s" %(self.atti.yaw))
        

if __name__ == '__main__':
    try:
        SubAtti()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass

