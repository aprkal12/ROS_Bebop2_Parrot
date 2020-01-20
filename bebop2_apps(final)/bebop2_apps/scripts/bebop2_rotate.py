#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import radians

DEFAULT_ANG_SPEED = 0.25
TARGET_ANGLE = 0.0

pi = 3.1415926535

class Rotate:
    
    def __init__(self):
    
        rospy.init_node('move_anglar_z', anonymous=True)
        
        self.pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
        
        rospy.sleep(1)
        
        self.tw     = Twist()
        self.Kp_ang = 1.0


    def rotate(self, ang):  # v = d / t,   t = d / v
        
        wise = 1.0
        
        if( ang < 0 ):
            wise = -1
        else:
            wise =  1
        
        duration = abs(ang) / DEFAULT_ANG_SPEED
        time2end = rospy.Time.now() + rospy.Duration(duration)
        
        self.tw.angular.z  = DEFAULT_ANG_SPEED * self.Kp_ang * wise;
        
        while(rospy.Time.now() < time2end):
            print( time2end - rospy.Time.now())
            self.pub.publish(self.tw)
        
        self.tw.linear.x = 0.0
        self.pub.publish(self.tw)
        
       
                
if __name__ == '__main__':

    try:
        target = float(input("input angle: "))
        TARGET_ANGLE = radians(target) # deg to rad
        
        r = Rotate()        
        r.rotate(TARGET_ANGLE)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:   pass
