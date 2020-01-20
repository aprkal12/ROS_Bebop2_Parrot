#!/usr/bin/env python

import rospy
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

        self.sub1 = rospy.Subscriber('/ar_pose_marker', AlvarMarkers, self.cbGetMarker )
        self.sub2 = rospy.Subscriber('/bebop/odom', Odometry, self.cbGetOdom, queue_size = 1)
        self.sub3 = rospy.Subscriber("bebop/states/ardrone3/PilotingState/AltitudeChanged",
                         Ardrone3PilotingStateAltitudeChanged,
                         self.cbGetAlt)
        self.pub  = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)

        self.rate      = rospy.Rate(10)

        self.tw        = Twist()
        self.msg       = AlvarMarker()
        self.msg_tmp   = AlvarMarker()

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
        ##############################
        self.is_cur_seq             = 0

        self.searching_middle_seq   = 0
        self.turn_front_seq         = 1
        self.align_center_seq       = 2
        self.go_forward_seq         = 3
        self.up_seq                 = 4
        self.finished_seq           = 5

        self.testflag1              = False
        self.testflag1_flag_1       = False
        self.testflag1_flag_2       = False
        self.seq1                   = 0
        self.seq2                   = 1
        self.seq3                   = 2
        self.test_seqs_finished     = False
        self.is_mark_checked        = False
        self.cnt                    = 0
        self.is_start               = False

        self.duration_2             = 0.0
        self.time2end_2             = 0.0

        while not rospy.is_shutdown():
            if self.test_seqs_finished == False and self.is_mark_checked == True:
                self.fnMain()

    def cbGetMarker(self,msg):
        n = len(msg.markers)
        if self.testflag1_flag_2 == False:
            if( n != 0 ):
                for tag in msg.markers:
                    if(tag.id == TARGET_ID):
                        self.msg = tag
                        # '''
                        if self.cnt >= 1 and self.is_start == True:
                            if self.testflag1_flag_1 == False:
                                self.testflag1_flag_1 = True
                                self.th_odom_1 = self.th_odom
                                print("First marker checked")
                        # '''
                        print("target marker ID checked!")

                #self.id = msg.markers[0].id
                #self.p  = msg.markers[0].pose
            else:
                print("marker not found")
                if self.testflag1_flag_1 == True:
                    self.testflag1_flag_2 = True
                    self.th_odom_2 = self.th_odom
                    print("second marker checked")
        else:
            for tag in msg.markers:
                if(tag.id == TARGET_ID):
                    self.msg_tmp = tag

        self.is_mark_checked = True

    def cbGetOdom(self,msg):
        theta = self.fnGetArPose(msg.pose.pose, 2)

        if  (theta >  5.):
            self.th_odom = theta - 2 * pi
        elif(theta < -5.):
            self.th_odom = theta + 2 * pi
        else:
            self.th_odom = theta

    def cbGetAlt(self,msg):
        self.current_alt = msg.altitude

    def fnMain(self):
        if self.is_cur_seq == self.searching_middle_seq:
            #print("searching middle~")
            self.is_seq_finished = self.fnSearchingMiddle()

            if self.is_seq_finished == True:
                print("Finish 1")
                self.is_cur_seq = self.turn_front_seq
                self.is_seq_finished = False

        elif self.is_cur_seq == self.turn_front_seq:
            print("turn front~")
            self.is_seq_finished = self.fnTurnFront()

            if self.is_seq_finished == True:
                print("Finish 2")
                self.is_cur_seq = self.align_center_seq
                self.is_seq_finished = False

        elif self.is_cur_seq == self.align_center_seq:
            print("align_center~")
            self.is_seq_finished = self.fnAlignCenter()

            if self.is_seq_finished == True:
                print("Finish 3")
                self.is_cur_seq = self.go_forward_seq
                self.is_seq_finished = False

        elif self.is_cur_seq == self.go_forward_seq:
            print("go forward~")
            self.is_seq_finished = self.fnGoForward()

            if self.is_seq_finished == True:
                print("Finish 4")
                self.is_cur_seq = self.up_seq
                self.is_seq_finished = False

        elif self.is_cur_seq == self.up_seq:
            print("up~~~")
            self.is_seq_finished = self.fnUp()

            if self.is_seq_finished == True:
                print("Finish 5")
                self.is_cur_seq = self.finished_seq
                self.is_seq_finished = False

        elif self.is_cur_seq == self.finished_seq:
            print("finish~~")
            self.test_seqs_finished = True

    def fnUp(self):
        self.fnChangeAlt(1.5)
        rospy.sleep(1)
        # back
        self.fnMoveX(1*-2)
        return True

    def fnGoForward(self):
        self.fnChangeAlt(0.9)
        rospy.sleep(1)
        self.fnMoveX(self.dist_x*0.8)
        rospy.sleep(1)

        return True

    def fnAlignCenter(self):

        self.fnChangeDirection()
        self.fnMoveY(self.dist_y)
        rospy.sleep(1)

        return True

    def fnTurnFront(self):
        print("goal = %f" % self.th_odom_mid)

        self.dist = self.msg_tmp.pose.pose.position.z

        theta = self.fnGetArPose(self.msg_tmp.pose.pose, 1)

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

        self.dist_x = self.dist * cos(self.theta)
        self.dist_y = self.dist * sin(self.theta)
        print("dist_y = %f " % self.dist_y)
        print("dist_x = %f " % self.dist_x)

        print("theta = %f " % degrees(self.theta))

        # ======================================

        self.duration = 0.0
        self.time2end = 0.0

        print("moving~~")

        if( self.theta < 0 ):
            self.wise =  1
        else:
            self.wise = -1

        if self.duration == 0.0 and self.time2end == 0.0:
            # self.TurnTh = radians(90) - self.TurnTh
            self.duration = abs(self.theta) / DEFAULT_ANG_SPEED
            self.time2end = rospy.Time.now() + rospy.Duration(self.duration)

            self.Kp_ang   = 0.575    # 0.575
            self.tw.angular.z  = DEFAULT_ANG_SPEED * self.Kp_ang * self.wise

        self.Kp_ang = 1.0
        while(rospy.Time.now() < self.time2end):
            self.pub.publish(self.tw)

        print("moving end")
        self.tw.angular.z = 0.0
        self.pub.publish(self.tw)

        return True

    def fnSearchingMiddle(self):
      
        if self.testflag1_flag_1 == False:
            self.fnTurn()
        elif self.testflag1_flag_2 == False:
            self.fnTurn()
        else:
            self.fnStop()
            self.fnChangeDirection()
            self.fnCalcMiddle()
            self.fnGoToMiddle()
            self.fnStop()
            rospy.sleep(1)
            return True
            

    def fnGetArPose(self,msg,rpy):
        q = (msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        theta = euler_from_quaternion(q)[rpy]   # case of marker rpy=1, case of odom rpy=2
        #if rpy set_middlea = theta - pi * 2
        #if rpy == 1:
           # print("1 touch theta = %f " % (theta/0.0174533 - 15.0))
        #pos_x = msg.pose.pose.position.z
        #pos_y = msg.pose.pose.position.y
        return theta

    def fnChangeAlt(self, target):
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

    def fnCalcMiddle(self):
        self.th_odom_mid = abs(abs(self.th_odom_2) - abs(self.th_odom_1)) / 2

    def fnChangeDirection(self):
        self.wise = self.wise * -1

    def fnMoveX(self, dist):
        #print("goal dist_x = %f" % (dist))
        direction = 1.0

        if( dist < 0 ):
            direction = -1
        else:
            direction =  1

        duration = abs(dist) / DEFAULT_LIN_SPEED
        time2end = rospy.Time.now() + rospy.Duration(duration)

        #self.tw.linear.y  = DEFAULT_LIN_SPEED * self.Kp * direction
        self.tw.linear.x  = DEFAULT_LIN_SPEED * self.Kp_lin * direction

        while(rospy.Time.now() < time2end):
            #print( time2end - rospy.Time.now())
            self.pub.publish(self.tw)

        #self.tw.linear.y = 0.0
        self.tw.linear.x = 0.0
        self.pub.publish(self.tw)

    def fnMoveY(self, dist):
        #self.wise = self.wise * -1
        duration = abs(dist) / DEFAULT_LIN_SPEED
        time2end = rospy.Time.now() + rospy.Duration(duration)

        self.tw.linear.y  = DEFAULT_LIN_SPEED * self.Kp_lin * self.wise #direction;

        while(rospy.Time.now() < time2end):
            #print( time2end - rospy.Time.now())
            self.pub.publish(self.tw)

        self.tw.linear.y = 0.0
        self.pub.publish(self.tw)

    def fnTurn(self):
        self.cnt+=1
        if self.cnt == 1:
            if self.msg.pose.pose.position.x != 0.0:
                print("setting~~")
                print self.msg.pose.pose.position
                if self.duration_2 == 0.0 and self.time2end_2 == 0.0:
                    self.duration_2 = abs(radians(100)) / DEFAULT_ANG_SPEED
                    self.time2end_2 = rospy.Time.now() + rospy.Duration(self.duration_2 * 0.2)
                
                    #self.Kp_ang   = 1.7    # 0.575
                    self.tw.angular.z  = DEFAULT_ANG_SPEED * 8 * self.wise
                
                #self.Kp_ang = 1.7
                while(rospy.Time.now() < self.time2end_2):
                    #print(rospy.Time.now() - self.time2end)
                    self.pub.publish(self.tw)
                print("end setting~~")

            self.is_start = True

        else:
            self.tw.angular.z = DEFAULT_ANG_SPEED * self.wise
            self.pub.publish(self.tw)

    def fnStop(self):
        self.tw.angular.z = 0.0
        self.pub.publish(self.tw)

    def fnGoToMiddle(self):
        if self.duration == 0.0 and self.time2end == 0.0:
            self.duration = abs(self.th_odom_mid) / DEFAULT_ANG_SPEED
            self.time2end = rospy.Time.now() + rospy.Duration(self.duration)
            self.Kp_ang = 0.751
            self.tw.angular.z  = DEFAULT_ANG_SPEED * self.Kp_ang * self.wise

            while(rospy.Time.now() < self.time2end):  # turn
                theta = self.fnGetArPose(self.msg.pose.pose, 1)
                            
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

if __name__ == '__main__':
    try:
        TARGET_ID = int(input("input marker ID: "))
        Align2Marker()

        rospy.spin()

    except rospy.ROSInterruptException:  pass
