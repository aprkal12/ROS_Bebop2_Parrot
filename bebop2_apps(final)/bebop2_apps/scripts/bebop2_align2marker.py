#!/usr/bin/env python
 
import rospy
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers
from math import pow, sqrt, atan, pi, radians, degrees, cos, sin
from tf.transformations import euler_from_quaternion #, quaternion_from_euler

DEFAULT_LIN_SPEED = 1

DEFAULT_ANG_SPEED = 0.125

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

class Align2Marker:

    def __init__(self):
    
        rospy.init_node('align_to_marker', anonymous = True)
        
        self.sub1 = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker )
        self.sub2 = rospy.Subscriber('/bebop/odom', Odometry, self.get_odom, queue_size = 1)
        self.pub  = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
        
        self.rate      = rospy.Rate(10)

        self.tw     = Twist()
        
        self.target_id = -1
        self.theta     =  0
        self.th_odom   =  0
        self.dist      =  0
        self.dist_x    =  0
        self.dist_y    =  0
        self.wise      =  1
        self.dir_x     =  1
        self.dir_y     =  1
        self.p         = Pose
        self.Kp_lin    = 1.0
        self.Kp_ang    = 1.0
        
        self.th_1      = 0.0
        self.th_2      = 0.0
        self.TurnTh    = 0.0
        self.TurnTh_nine = 0.0
        self.duration  = 0.0
        self.time2end  = 0.0
        
        self.marker_found_1st = False
        self.marker_found_2nd = False
        self.marker_align_finish = False
                
        
    def get_odom(self, msg):
        #self.p = msg.pose.pose
                    
        theta = self.get_ar_pose(msg.pose.pose, 2)
        
        if  (theta >  5.):
            self.th_odom = theta - 2 * pi            
        elif(theta < -5.):
            self.th_odom = theta + 2 * pi
        else:
            self.th_odom = theta
        
        
    def get_marker(self, msg):
    
        if( self.marker_found_2nd == False):
            
            n = len(msg.markers)
            
            if( n == 0 ):
                if( self.marker_found_1st == False):
                    #finding marker
                    self.tw.angular.z = 0.125
                    self.pub.publish(self.tw)
                
                else:
                    if( self.marker_found_2nd == False ):
                        self.marker_found_2nd = True
                        print('!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!')
                        self.tw.angular.z = 0.0
                        self.pub.publish(self.tw)
                    
                        self.th_2 = self.th_odom
                        print("marker 2nd found!")
                        print("theta1 = %f, theta2 = %f" %(self.th_1, self.th_2))

                    else:
                        pass
            
            else:# this means markers found
                for tag in msg.markers:
                
                    if(tag.id == TARGET_ID):
                        print("target marker ID checked!")
                        self.pub.publish(self.tw)
                        if( self.marker_found_1st == False ):
                            self.marker_found_1st = True
                            self.th_1 = self.th_odom
                            print("marker 1st found!")
                        
                        
                        self.dist = tag.pose.pose.position.z
                        '''
                        theta = self.get_ar_pose(tag.pose.pose, 1)
                        
                        if  (theta >  5.):
                            self.theta = theta - 2 * pi            
                        elif(theta < -5.):
                            self.theta = theta + 2 * pi
                        else:
                            self.theta = theta
                        
                        self.dist_x = self.dist * cos(theta)
                        self.dist_y = self.dist * sin(theta)
                        '''    
        elif( self.marker_align_finish == False): # align_center
        
            #print('!!!!!!!!!!!! turn  !!!!!!!!!!!!!')
            
            self.TurnTh = (self.th_2-self.th_1) /2
            
            wise = 1.0
                
            if( self.TurnTh < 0 ):
                wise = 1
            else:
                wise =  -1
                
            if self.duration == 0.0 and self.time2end == 0.0:
                # self.TurnTh = radians(90) - self.TurnTh 
                self.duration = abs(self.TurnTh) / DEFAULT_ANG_SPEED
                self.time2end = rospy.Time.now() + rospy.Duration(self.duration)
                #self.Kp_ang = 0.575
                self.Kp_ang = 0.75
                self.tw.angular.z  = DEFAULT_ANG_SPEED * self.Kp_ang * wise;
            self.Kp_ang = 1.0
            
            while(rospy.Time.now() < self.time2end):
                #print("time2end : ", time2end, "  Time.now() : ", rospy.Time.now())
                #print(self.time2end)
                #print(rospy.Time.now())
                print('=============== %.2f   %.2f =================' % (self.marker_found_1st, self.marker_found_2nd))
                #print( time2end - rospy.Time.now())
                self.pub.publish(self.tw)
                
            
            
            self.tw.angular.z = 0.0
            self.pub.publish(self.tw)
            self.marker_align_finish = True
            self.turn_forward()
            self.move_forward()
        else:
            pass
                    
    def turn_forward(self): # align to center
        self.duration = 0.0
        self.time2end = 0.0
        
        print("welcome to forward")
        
        self.TurnTh_nine = (radians(90) - self.TurnTh) / 2
        wise = 1.0
                
        if( self.TurnTh < 0 ):
            wise = 1
        else:
            wise =  -1
            
        if self.duration == 0.0 and self.time2end == 0.0:
            # self.TurnTh = radians(90) - self.TurnTh 
            self.duration = abs(self.TurnTh) / DEFAULT_ANG_SPEED
            self.time2end = rospy.Time.now() + rospy.Duration(self.duration)
            #self.Kp_ang = 0.575
            self.Kp_ang = 0.75
            self.tw.angular.z  = DEFAULT_ANG_SPEED * self.Kp_ang * wise;
        self.Kp_ang = 1.0
            
        while(rospy.Time.now() < self.time2end):
                #print("time2end : ", time2end, "  Time.now() : ", rospy.Time.now())
                print(self.time2end)
                print(rospy.Time.now())
                print('===============', abs(self.TurnTh), '=================')
                #print( time2end - rospy.Time.now())
                self.pub.publish(self.tw)
                
        print("??????????????????????????????????")
        self.tw.angular.z = 0.0
        self.pub.publish(self.tw)
    
    '''def move_forward(self): # move to AR marker
        dist = sin(degrees(self.TurnTh)) * self.dist
        print("self-dist: %f, dist: %f" % (self.dist, dist))
        direction = 1.0
        
        if( dist < 0 ):
            direction = -1
        else:
            direction =  1
        
        duration = abs(dist) / DEFAULT_LIN_SPEED
        time2end = rospy.Time.now() + rospy.Duration(duration)
        
        self.tw.linear.y  = DEFAULT_LIN_SPEED * self.Kp_lin * direction;
        
        while(rospy.Time.now() < time2end):
            print( time2end - rospy.Time.now())
            self.pub.publish(self.tw)    
        
        self.tw.linear.y = 0.0
        self.pub.publish(self.tw)
     '''   
    def get_ar_pose(self, msg, rpy):   # case of marker rpy=1, case of odom rpy=2
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
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
             
        theta = euler_from_quaternion(q)[rpy]   # case of marker rpy=1, case of odom rpy=2
        
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
        Align2Marker()
        rospy.spin()
        
    except rospy.ROSInterruptException:  pass

