#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

DEFAULT_LIN_SPEED = 0.25
# 0.25 = default
TARGET_DIST = 0.0


class Move:
    
    def __init__(self):
    
        rospy.init_node('move_linear_x', anonymous=True)
        
        self.pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
        
        rospy.sleep(1)
        
        self.tw = Twist()
        self.Kp   = 1.0


    def move_x(self, dist):  # v = d / t,   t = d / v
        
        direction = 1.0
        
        if( dist < 0 ):
            direction = -1
        else:
            direction =  1
        1
        duration = abs(dist) / DEFAULT_LIN_SPEED
        time2end = rospy.Time.now() + rospy.Duration(duration)
        
        self.tw.linear.y  = DEFAULT_LIN_SPEED * self.Kp * direction;
        
        while(rospy.Time.now() < time2end):
            print( time2end - rospy.Time.now())
            self.pub.publish(self.tw)    
        
        self.tw.linear.y = 0.0
        self.pub.publish(self.tw)
        
       
                
if __name__ == '__main__':

    try:
        TARGET_DIST = float(input("input distance to fly: "))
        
        m = Move()        
        m.move_x(TARGET_DIST)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:   pass
'''
        
    def rotate(self, direction, ang):
    
        ang = radians(ang)
        
        duration = ang / 2 / DEFAULT_ANG_SPEED * 1.15
        time2end = rospy.Time.now() + rospy.Duration(duration)
        
        self.t.angular.z  = DEFAULT_ANG_SPEED;
        
        if(direction != 1):
            self.t.angular.z = -self.t.angular.z
        
        while(rospy.Time.now() < time2end):
            print( time2end - rospy.Time.now())
            self.pb_twist.publish(self.t)   
        
        self.t.angular.z = 0.0
        self.pb_twist.publish(self.t)
'''
