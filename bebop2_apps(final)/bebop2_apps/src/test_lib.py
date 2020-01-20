#! /usr/bin/env python
 
from py_lib.DistanceGPS import DistanceGPS
#from py_lib.GetGPSInfo import GetGPSInfo 
import rospy
from std_msgs.msg import String
from bebop_msgs.msg import Ardrone3GPSStateNumberOfSatelliteChanged
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged


is_satellite_check = False
first_latitude = 0.0
first_longitude = 0.0

def cbSatelliteNumCheck(msg):
    n = msg.numberOfSatellite
    print("number of satellite = " + str(n))

    if n >= 10:
        is_satellite_check = True

def fnCalAzi(lat_p1, lon_p1, lat_p2, lon_p2):
    distance = gps.get_distance(lat_p1, lon_p1, lat_p2, lon_p2)
    bearing  = gps.get_bearing( lat_p1, lon_p1, lat_p2, lon_p2)
    print("distance = %f, bearing = %f." %(distance, bearing))
        

def cbFirstLocationSave(msg):
    global first_latitude
    global first_longitude
    if msg.latitude != 500.0 and msg.longitude != 500.0:
        latitude = msg.latitude
        longitude = msg.longitude
        print("latitude = %f, longitude = %f" % (latitude, longitude))

        if first_latitude == 0.0 and first_longitude == 0.0 and is_satellite_check == True:
            first_latitude = latitude
            first_longitude = longitude
            fnCalAzi(first_latitude, first_longitude, 35.944029, 126.684297)
            print("First loaction save!")
    else:
        print("No GPS Signal")

if __name__ == '__main__':
    try:
        gps = DistanceGPS()

        rospy.init_node("get_gps", anonymous=10)

        rospy.Subscriber("bebop/states/ardrone3/GPSState/NumberOfSatelliteChanged", Ardrone3GPSStateNumberOfSatelliteChanged, cbSatelliteNumCheck)
        rospy.Subscriber("bebop/states/ardrone3/PilotingState/PositionChanged", Ardrone3PilotingStatePositionChanged, cbFirstLocationSave)
        
        rospy.spin()

    except KeyboardInterrupt:
        pass


