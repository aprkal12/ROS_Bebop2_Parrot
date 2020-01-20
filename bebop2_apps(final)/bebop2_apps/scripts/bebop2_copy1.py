#!/usr/bin/env python

import rospy
import requests
from std_msgs.msg import Empty
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from ar_track_alvar_msgs.msg import AlvarMarkers, AlvarMarker
from math import pow, sqrt, atan, pi, radians, cos, sin, degrees
from tf.transformations import euler_from_quaternion #, quaternion_from_euler
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged

DEFAULT_ANG_SPEED = 0.125
DEFAULT_LIN_SPEED = 0.25

TARGET_ID = -1

class Align2Marker:

    def __init__(self):

        rospy.init_node('align_to_marker', anonymous = True)

        self.sub1 = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.get_marker )
        self.sub2 = rospy.Subscriber('/bebop/odom', Odometry, self.get_odom, queue_size = 1)
        self.sub3 = rospy.Subscriber("bebop/states/ardrone3/PilotingState/AltitudeChanged", 
                         Ardrone3PilotingStateAltitudeChanged, 
                         self.get_alt)
        self.pub  = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)

        self.rate      = rospy.Rate(10)

        self.tw     = Twist()
        self.msg    = AlvarMarker()
        self.msg_tmp    = AlvarMarker()

        self.target_id = -1
        self.theta     = 0.0
        self.th_odom   = 0.0
        self.dist      = 0.0
        self.dist_x    = 0.0
        self.dist_y    = 0.0
        self.dir_x     = 1.0
        self.dir_y     = 1.0
        self.p         = Pose
        self.Kp_ang    = 1.0
        self.Kp_lin    = 0.28

        self.duration  = 0.0
        self.time2end  = 0.0
        self.wise      = 1.0

        self.marker_found_1st = False
        self.marker_found_2nd = False
        self.marker_align_finish = False

        # step 1
        self.is_arrive_middle     = False
        self.is_odom_received     = False
        self.is_marker_found      = False
        self.first_found_marker   = False
        self.second_found_marker  = False
        self.th_odom_1            = 0.0
        self.th_odom_2            = 0.0
        self.th_odom_mid          = 0.0

        # step 2 (test)
        self.theta_1              = 0.0
        self.theta_2              = 0.0
        self.theta_mid            = 0.0
        #self.is_faced_marker      = False
        self.current_alt          = 1.0
		

        # test flag
        self.is_msg               = False
        
        self.is_mark_checked      = False     
        self.cnt                 = 0
        self.tck                 = 0
        self.is_start           = False
        self.duration_2 = 0.0
        self.time2end_2 = 0.0

        print('init')

        #if self.gps_finished == True:
            # GPS 
        self.change_alt(1.1)
        while not rospy.is_shutdown():
            if self.is_arrive_middle == False and self.is_mark_checked == True:
                self.set_middle()

    def test(self):
        print('!!!!!!!!!!!!!!!!!!!!')
        cnt = 0
        while True:
            cnt+=1
            if cnt > 1000000:
                break

    def get_alt(self, msg):
        self.current_alt = msg.altitude

        #print("current height = %f" %(self.current_alt))       

    def get_odom(self, msg):
        if self.is_odom_received == False:
            self.is_odom_received = True

        theta = self.get_ar_pose(msg.pose.pose, 2)

        if  (theta >  5.):
            self.th_odom = theta - 2 * pi
        elif(theta < -5.):
            self.th_odom = theta + 2 * pi
        else:
            self.th_odom = theta

    def get_marker(self, msg):
        n = len(msg.markers)
        if self.second_found_marker == False:
            if( n != 0 ):
                for tag in msg.markers:
                    if(tag.id == TARGET_ID):
                        self.msg = tag
                        #'''
                        
                        # '''
                        if self.cnt >= 1 and self.is_start == True:    
                            if self.first_found_marker is False:
                                self.first_found_marker = True
                                self.th_odom_1 = self.th_odom
                                print("marker First_found")

                #self.id = msg.markers[0].id
                #self.p  = msg.markers[0].pose


            else:
                if self.first_found_marker is True:
                    self.second_found_marker = True
                    self.th_odom_2 = self.th_odom
                    print("marker Second_found")

        else:
            for tag in msg.markers:
                if(tag.id == TARGET_ID):
                    self.msg_tmp = tag

        self.is_mark_checked = True

    def get_ar_pose(self, msg, rpy):   # case of marker rpy=1, case of odom rpy=2
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
             
        theta = euler_from_quaternion(q)[rpy]   # case of marker rpy=1, case of odom rpy=2
        #if rpy set_middlea = theta - pi * 2
        #if rpy == 1:
           # print("1 touch theta = %f " % (theta/0.0174533 - 15.0))
        #pos_x = msg.pose.pose.position.z
        #pos_y = msg.pose.pose.position.y

        return theta
        
    def set_middle(self):
        #print("set_middle: ", self.current_alt)
        if self.first_found_marker == False:
            self.turn()
        elif self.second_found_marker == False:
            self.turn()
        else:
            self.stop()
            self.change_direction()
            self.calc_middle()
            rospy.sleep(1)
            self.go_to_middle()
            self.stop()
			
            rospy.sleep(1)

            print(self.th_odom_1, self.th_odom_2, self.th_odom_mid)
            self.dist = self.msg_tmp.pose.pose.position.z
            theta = self.get_ar_pose(self.msg_tmp.pose.pose, 1)
            print(self.dist, self.theta)
            if  (theta >  5.0):
                self.theta = theta - 2*pi
            elif(theta < -5.0):
                self.theta = theta + 2*pi
            else:
                self.theta = theta
                
            if  ( self.theta > 0.261799):
                self.theta = self.theta + 0.261799    
            elif( self.theta < -0.261799):
                self.theta = self.theta - 0.261799 
            else:
                pass

            self.dist_x = self.dist * cos(self.theta)
            self.dist_y = self.dist * sin(self.theta)
            print("dist_y = %f " % self.dist_y)
            print("dist_x = %f " % self.dist_x)

            print("theta = %f " % degrees(self.theta))
            
            print("dist_y = %f " % self.dist_y)
            print("theta = %f " % degrees(self.theta))
            '''
            self.theta_mid = abs(abs(self.theta_2) - abs(self.theta_1)) /2.0

            print("odom_theta = %f" % (self.th_odom_mid))
            print("theta3      = %f" % degrees(self.theta_mid))
            '''
            if self.dist > 2.0:
                print("cur dist = %f" % self.dist)

            self.align_forward()
            self.move_y(self.dist_y*1.6)
            rospy.sleep(1)
            self.change_alt(1.0)
            #print("down: ", self.current_alt)
            rospy.sleep(1)
            print('====================', self.dist)			
            self.move_x(self.dist*0.6)
            #self.move_x(self.dist_x * 0.5) # require value fix
            rospy.sleep(1)
			
            self.change_alt(1.5)
            self.move_x(1 * 0.6 * -2)
            self.is_arrive_middle = True

        #print(self.th_odom_1, self.th_odom_2, self.th_odom_mid)
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
    # ''' move_x tests
    def move_x(self, dist):  # v = d / t,   t = d / v
        print("goal dist_x = %f" % (dist))
        direction = 1.0
        
        if( dist < 0 ):
            direction = -1
        else:
            direction =  1
        
        duration = abs(dist) / DEFAULT_LIN_SPEED
        time2end = rospy.Time.now() + rospy.Duration(duration)
        
        #self.tw.linear.y  = DEFAULT_LIN_SPEED * self.Kp * direction
        self.tw.linear.x  = DEFAULT_LIN_SPEED *self.Kp_lin * direction
        
        while(rospy.Time.now() < time2end):
            #print( time2end - rospy.Time.now())
            self.pub.publish(self.tw)    
        
        #self.tw.linear.y = 0.0
        self.tw.linear.x = 0.0
        self.pub.publish(self.tw)
    # '''
    def move_y(self, dist):  # v = d / t,   t = d / v
        print("!!!!!!!!!!!!!!"+ str(dist) +"????????????????????")
        self.wise = self.wise * -1
        '''direction = 1.0
        
        if( dist < 0 ):
            direction = 1
        else:
            direction = -1
        '''

        duration = abs(dist) / DEFAULT_LIN_SPEED
        time2end = rospy.Time.now() + rospy.Duration(duration)
        
        self.tw.linear.y  = DEFAULT_LIN_SPEED * self.Kp_lin * self.wise #direction;
        
        while(rospy.Time.now() < time2end):
            #print( time2end - rospy.Time.now())
            self.pub.publish(self.tw)    
        
        self.is_faced_marker = True
        self.tw.linear.y = 0.0
        self.pub.publish(self.tw)

    def align_forward(self): # align_forward use theta
        self.duration = 0.0
        self.time2end = 0.0

        if( self.theta < 0 ):
            self.wise =  1
        else:
            self.wise = -1

        if self.duration == 0.0 and self.time2end == 0.0:
            # self.TurnTh = radians(90) - self.TurnTh 
            self.duration = abs(self.theta) / DEFAULT_ANG_SPEED
            self.time2end = rospy.Time.now() + rospy.Duration(self.duration)
            
            self.Kp_ang   = 0.575    # 0.575theta
            self.tw.angular.z  = DEFAULT_ANG_SPEED * self.Kp_ang * self.wise

        while(rospy.Time.now() < self.time2end):
            self.pub.publish(self.tw)

        print("stop")
        self.tw.angular.z = 0.0
        self.pub.publish(self.tw)

    def go_to_middle(self):
        print('go to middle!')
        if self.duration == 0.0 and self.time2end == 0.0:
            self.duration = abs(self.th_odom_mid) / DEFAULT_ANG_SPEED
            self.time2end = rospy.Time.now() + rospy.Duration(self.duration)
            self.Kp_ang = 0.751
            self.tw.angular.z  = DEFAULT_ANG_SPEED * self.Kp_ang * self.wise

        #self.Kp_ang = 1
        while(rospy.Time.now() < self.time2end):  # turn
            theta = self.get_ar_pose(self.msg.pose.pose, 1)
                        
            if  (theta >  5.0):
                self.theta = theta - 2*pi            
            elif(theta < -5.0):
                self.theta = theta + 2*pi
            else:
                self.theta = theta
                
            if  ( self.theta > 0.261799):
                self.theta = self.theta + 0.261799    
            elif( self.theta < -0.261799):
                self.theta = self.theta - 0.261799
            else:    pass
            
            self.pub.publish(self.tw)


    def calc_middle(self):
        self.th_odom_mid = abs(abs(self.th_odom_2) - abs(self.th_odom_1)) /2

    def change_direction(self):
        self.wise = self.wise * -1

    def turn(self):
        self.cnt += 1
        if(self.cnt == 1):
            print self.msg.pose.pose.position
            if self.msg.pose.pose.position.x != 0.0:
                print('turn!!')
                print self.msg.pose.pose.position
                if self.duration_2 == 0.0 and self.time2end_2 == 0.0:
                    self.duration_2 = abs(radians(100)) / DEFAULT_ANG_SPEED
                    self.time2end_2 = rospy.Time.now() + rospy.Duration(self.duration_2)
                
                    #self.Kp_ang   = 1.7    # 0.575
                    self.tw.angular.z  = DEFAULT_ANG_SPEED * self.Kp_ang * self.wise
                
                #self.Kp_ang = 1.7
                while(rospy.Time.now() < self.time2end_2):
                    #print(rospy.Time.now() - self.time2end)
                    self.pub.publish(self.tw)
                print('finish turn')

            
            self.is_start = True
            
        self.tw.angular.z = DEFAULT_ANG_SPEED * self.wise
        self.pub.publish(self.tw)

    def stop(self):
        self.tw.angular.z = 0.0
        self.pub.publish(self.tw)
    
    # =================================================================================

if __name__ == '__main__':
    try:
        TARGET_ID = int(input("input marker ID: "))
        
        Align2Marker()
        rospy.spin()

    except rospy.ROSInterruptException:  pass
