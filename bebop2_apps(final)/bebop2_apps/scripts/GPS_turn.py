#! /usr/bin/env python
 
from py_lib.DistanceGPS import DistanceGPS
#from py_lib.GetGPSInfo import GetGPSInfo 
import rospy
import requests
from py_lib.GetChar import GetChar
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from math import radians
from bebop_msgs.msg import Ardrone3GPSStateNumberOfSatelliteChanged
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged
from bebop_msgs.msg import Ardrone3PilotingStateAttitudeChanged
from bebop_msgs.msg import Ardrone3PilotingStateAltitudeChanged

DEFAULT_LIN_SPEED = 0.25

# Add by mansu(start)
# TARGET_LA T = 35.9440192
# TARGET_LON = 126.6843312
# Add by mansu(end)

class Goto:
    def __init__(self):
        rospy.init_node("get_gps", anonymous=10)

        self.pub = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
        self.sub1 = rospy.Subscriber("bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged", Ardrone3GPSStateNumberOfSatelliteChanged, self.cbSatelliteNumCheck, queue_size=1)
        self.sub2 = rospy.Subscriber("bebop/states/ardrone3/PilotingState/PositionChanged", Ardrone3PilotingStatePositionChanged, self.cbFirstLocationSave, queue_size=1, buff_size=2**24)
        self.sub3 = rospy.Subscriber("bebop/states/ardrone3/PilotingState/AttitudeChanged", Ardrone3PilotingStateAttitudeChanged, self.cbGetAzi, queue_size=1)
        self.sub4 = rospy.Subscriber("bebop/states/ardrone3/PilotingState/AltitudeChanged", Ardrone3PilotingStateAltitudeChanged, self.cbGetAlt)

        self.is_satellite_check = False
        self.first_latitude = 0.0
        self.first_longitude = 0.0
        
        self.cur_latitude = 0.0
        self.cur_longitude = 0.0

        self.cur_azi = 0.0
        self.tw = Twist()
        self.distance = 0.0
        self.bearing = 0.0

        self.Kp   = 0.5
        self.rate = rospy.Rate(10)
        self.current_alt = 0
        
    #def 

    def fnchange_yaw(self,target): 
        state_round = 0
        
        if( self.cur_azi >= 0 ):
            if( (self.cur_azi > radians(target)) and (self.cur_azi - 3.1415926535 < radians(target)) ):
                state_round = -1
            else:
                state_round = 1
        else:
            if( (self.cur_azi < radians(target)) and (self.cur_azi + 3.1415926535 > radians(target)) ):
                state_round = 1
            else:
                state_round = -1
        
        if( state_round == 1 ):
            self.tw.angular.z =  -0.2
        elif(state_round == -1):
            self.tw.angular.z =  0.2
        
        while( not( ((self.cur_azi + 0.1 > radians(target)) and (self.cur_azi - 0.1 < radians(target))) )):
            self.pub.publish(self.tw)

        self.tw.angular.z = 0.0
        self.pub.publish(self.tw) 

    def fnCalAzi(self,lat_p1, lon_p1, lat_p2, lon_p2):

        gps = DistanceGPS()
        self.distance = gps.get_distance(lat_p1, lon_p1, lat_p2, lon_p2)
        self.bearing  = gps.get_bearing( lat_p1, lon_p1, lat_p2, lon_p2)
        print("distance = %f, bearing = %f." %(self.distance, self.bearing))
        self.fnchange_yaw(self.bearing)
            

    def cbGetAlt(self, msg):
        self.current_alt = msg.altitude

    def cbGetAzi(self,msg):
        
        self.cur_azi = msg.yaw
    def cbSatelliteNumCheck(self, msg):
        n = msg.numberOfSatellite
        #print("number of satellite = " + str(n))

        if n >= 10:
            self.is_satellite_check = True

    def cbFirstLocationSave(self, msg):
        gps = DistanceGPS()
        self.cur_latitude = msg.latitude
        self.cur_longitude = msg.longitude

        self.distance = gps.get_distance(msg.latitude, msg.longitude, TARGET_LAT, TARGET_LON)#35.944029, 126.684297)
        print("latitude = %f, longitude = %f, distance = %f" % (self.cur_latitude, self.cur_longitude, self.distance))


    def move_x(self, des_lat, des_lon):
        gps = DistanceGPS()
        count = 0
        print('move !!!')
        self.tw.linear.x  = DEFAULT_LIN_SPEED * self.Kp
        self.pub.publish(self.tw)    
        
        #print(self.distance , self.tw.linear.x)
        
        while not rospy.is_shutdown():
            print("moving...")
            count += 1
            #print(count)
            #print("=====  self.distance: %f" % self.distance)
            if(self.distance < 1.00003):#0.10003): default: 0.00003
                #print("vshort dist =====  self.distance: %f  self.cur_lat" % self.distance, self.cur_latitude)
                break
            elif(self.distance < 1.00007):#0.20007): default: 0.00007
                #print("short dist =====  self.distance: %f  self.cur_lat" % self.distance, self.cur_latitude)
                if(count >= 20000):
                    count = 0
                    #print(self.distance , self.tw.linear.x)
                    
                self.tw.linear.x  = DEFAULT_LIN_SPEED * self.Kp * 0.3
                self.pub.publish(self.tw)
                
            else:
                
                if(count >= 20000):
                    #print("long dist =====  self.distance: %f  self.cur_lat" % self.distance, self.cur_latitude)
                    count = 0
                    #print(self.distance , self.tw.linear.x)
                    target_yaw = gps.get_bearing(self.cur_latitude, self.cur_longitude,des_lat,des_lon) 
                    self.fnchange_yaw(target_yaw)
                    
                self.tw.linear.x  = DEFAULT_LIN_SPEED * self.Kp
                self.pub.publish(self.tw)   
                
            self.distance=gps.get_distance(self.cur_latitude, self.cur_longitude, des_lat,des_lon)

            #self.rate.sleep()
        #rospy.spin()
            
        print('end')
        self.tw.linear.x = 0.0
        self.pub.publish(self.tw)

    def change_alt(self, target):
        print('change alt!!!!!')
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
        # Add by mansu(start)
        '''r = requests.get("https://localhost:3000/get", verify=False)
        r = r.json()
        global TARGET_LAT
        TARGET_LAT = float(r["dest_gps2_alt"])
        global TARGET_LON
        TARGET_LON = float(r["dest_gps2_lat"])
        print(TARGET_LAT, TARGET_LON)'''
        # Add by mansu(end)
        goto = Goto()
        
        # comment by mansu(start)

        # goto.change_alt(3)
        while True:
            if goto.is_satellite_check == True:
                break
        
        print('i found satelite!')
        # goto.fnCalAzi(goto.cur_latitude, goto.cur_longitude, TARGET_LAT, TARGET_LON)#35.944029, 126.684297)
        # print("First loaction save!")
        # goto.move_x(TARGET_LAT, TARGET_LON)#35.944029, 126.684297)35.944029, 126.684297)
        # goto.change_alt(1)

        # key_input = GetChar()
        # while True:
        #     print('end!!!')
        #     key = key_input.getch()
        #     if key == 'q':
        #         break

        # comment by mansu(end)

        goto.change_alt(3)
        global HERE_LAT
        HERE_LAT = 35.943614#float(r["here_gps1_alt"])
        global HERE_LON
        HERE_LON = 126.683901#float(r["here_gps1_lat"])
        print(HERE_LAT, HERE_LON)
        goto.fnCalAzi(goto.cur_latitude, goto.cur_longitude, HERE_LAT, HERE_LON)
        #goto.change_alt(3)
        goto.move_x(HERE_LAT, HERE_LON)#35.944029, 126.684297)35.944029, 126.684297)
        
        # publish land

    except KeyboardInterrupt:
        pass


