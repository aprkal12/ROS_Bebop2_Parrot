#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged

pi = 3.1415926535

TARGET_ALTITUDE   = 1.0

class ChangeAlt:
    
    def __init__(self):
    
        rospy.init_node('change_altitude', anonymous=True)
        
        self.pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
        
        rospy.Subscriber("bebop/states/ardrone3/PilotingState/AltitudeChanged", 
                         Ardrone3PilotingStateAltitudeChanged, 
                         self.get_alt)
        rospy.sleep(1)
        
        self.tw = Twist()
        self.current_alt = 1.0
        
    def get_alt(self,msg):
        self.current_alt = msg.altitude
        print("current height = %f" %(self.current_alt))
    
    def change_alt(self, target):
    
        if(target > self.current_alt):
            self.tw.linear.z =  0.5
            
            while(target > self.current_alt):
                self.pub.publish(self.tw)
            
        elif(target < self.current_alt):
            self.tw.linear.z = -0.5
        
            while(target < self.current_alt):
                self.pub.publish(self.tw)
        
        self.tw.linear.z = 0.0
        self.pub.publish(self.tw) 
        
        print "reached target height!"
        
                
if __name__ == '__main__':

    try:
        TARGET_ALTITUDE = float(input("input target height: "))
        
        ca = ChangeAlt()        
        ca.change_alt(TARGET_ALTITUDE)
        
        rospy.spin()
        
    except rospy.ROSInterruptException:   pass
