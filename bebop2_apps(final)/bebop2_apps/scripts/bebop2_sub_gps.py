#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
#from tf.transformations import euler_from_quaternion #, quaternion_from_euler

class SubGPS:

    def __init__(self):
    
        rospy.init_node('SubGPS', anonymous = True)
        self.sub = rospy.Subscriber('/bebop/fix',NavSatFix , self.get_gps, queue_size = 1)
        
        self.rate = rospy.Rate(10)
        
        self.gps  =   NavSatFix      
        
    def get_gps(self, msg):
    
        self.gps = msg
        
        print("---")      
        print("latitude = %f" %(self.gps.latitude))
        print("longitude= %f" %(self.gps.longitude))
        

if __name__ == '__main__':
    try:
        gps=SubGPS()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass

