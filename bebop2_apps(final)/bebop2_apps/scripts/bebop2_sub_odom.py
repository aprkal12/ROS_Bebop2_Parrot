#!/usr/bin/env python

import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
#from tf.transformations import euler_from_quaternion #, quaternion_from_euler

class SubOdom:

    def __init__(self):
    
        rospy.init_node('sub_odom', anonymous = True)
        self.sub = rospy.Subscriber('/bebop/odom', Odometry, self.get_odom, queue_size = 1)
        
        self.rate = rospy.Rate(10)
        
        self.p    = Pose        
        
    def get_odom(self, msg):
    
        self.p = msg.pose.pose
        
        print("---")
        print("orientation.x = %f" %(self.p.orientation.x))
        print("orientation.y = %f" %(self.p.orientation.y))
        print("orientation.z = %f" %(self.p.orientation.z))
        print("orientation.w = %f" %(self.p.orientation.w))
        

if __name__ == '__main__':
    try:
        SubOdom()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass

