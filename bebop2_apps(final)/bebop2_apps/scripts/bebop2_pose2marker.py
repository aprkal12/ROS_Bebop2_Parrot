#!/usr/bin/env python
 
from __future__ import print_function
import rospy
from std_msgs.msg import Empty
from math import pow, sqrt, atan, pi, radians, cos, sin
from ar_track_alvar_msgs.msg import AlvarMarkers
from tf.transformations import euler_from_quaternion #, quaternion_from_euler

TARGET_ID = -1
        
'''           
      | marker  |  (0 < O)                             | marker  |  (0 > O)          
      -----+---------+                            ----------+-----
           |\R-0    R|                            |     R-0/|
           |0\       |                            |       /0|
           |  \      |                            |      /  |
           |   \     |                            |     /   |
           |  dist   |                            |  dist   |
    dist_x |     \   |                            |   /     |
           |      \  |                            |  /      |
           |       \0|                            | /       |
           |R    R-0\|          location          |/        |
           +---------x <<<<<<<     of     >>>>>>> x---------+
             dist.y              bebop2
                                         
                                
    dist   = pose.position.z 
    0      = euler_from_quaternion(q)[1]
    dist_x = dist * cos0
    dist_y = dist * sin0
'''

class Pose2Marker:

    def __init__(self):
    
        rospy.init_node('move_to_marker', anonymous = True)
        
        self.sb_mark   = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker )
        
        self.rate      = rospy.Rate(10)

        self.target_id = -1
        self.theta     =  0
        self.dist      =  0
        self.dist_x    =  0
        self.dist_y    =  0
        self.wise      =  1
        
        self.calcurate_complete = False
        
        
        
    def get_marker(self, msg):
    
        n = len(msg.markers)
        
        if( n != 0 ):# this means markers found
            print("marker found!")
            
            for tag in msg.markers:
            
                if(tag.id == TARGET_ID):
                    
                    print("target marker ID checked!")
                    
                    self.dist = tag.pose.pose.position.z
                    
                    theta = self.get_ar_pose(tag)
                    
                    if  (theta >  5.):
                        self.theta = theta - 2 * pi            
                    elif(theta < -5.):
                        self.theta = theta + 2 * pi
                    else:
                        self.theta = theta
                    
                    self.dist_x = self.dist * cos(theta)
                    self.dist_y = self.dist * sin(theta)
                        
                    self.print_info()
        
        else:
            print("Marker not found.")
    
    def get_ar_pose(self, msg):
        """
          orientation x,y,z,w --+
                                +--> 4   +-------------------------+
        input orientaion of marker ----->|                         |
                                         | euler_from_quaternion() |
        returnned rpy of marker <--------|                         |
                                 +-- 3   +-------------------------+
                 r,p,y angle <---+
                                         +------------+------------+
                                         |   marker   |   robot    |
                                         +------------+------------+
          r: euler_from_quaternion(q)[0] | roll   (x) | (y) pitch  |
        * p: euler_from_quaternion(q)[1] | pitch  (y) | (z) yaw ** | <-- 
          y: euler_from_quaternion(q)[2] | yaw    (z) | (x) roll   | 
                                         +------------+------------+
        """
        q = (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, 
             msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)
             
        theta = euler_from_quaternion(q)[1]
        
        if theta < 0:
            theta = theta + pi * 2
        if theta > pi * 2:
            theta = theta - pi * 2
        
        #pos_x = msg.pose.pose.position.z
        #pos_y = msg.pose.pose.position.y

        return theta
        
        
    def print_info(self):
        print("dist = %f, dist_x = %f, dist_y = %f, theta = %f" 
              %(self.dist, self.dist_x, self.dist_y, self.theta/0.0174533))
        

if __name__ == '__main__':
    try:
        TARGET_ID = int(input("input marker ID: "))
        Pose2Marker()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass

