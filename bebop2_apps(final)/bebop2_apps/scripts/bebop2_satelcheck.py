#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from bebop_msgs_msg import Ardrone3GPSStateNumberOfSatelliteChanged
from bebop_msgs.msg import Ardrone3PilotingStatePositionChanged

is_satellite_check = False
first_latitude = 0.0
first_longitude = 0.0

def cbSatelliteNumCheck(msg):
    n = msg.numberOfSatellite
    print("number of satellite = " + str(n))

    if n >= 10:
        is_satellite_check = True

def cbFirstLocationSave(msg):
    if msg.latitude != 500.0 and msg.longitude != 500.0:

        latitude = msg.latitude
        longitude = msg.longitude
        print("latitude = %f, longitude = %f" % (latitude, longitude))

        if first_latitude == 0.0 and first_longitude == 0.0:
            first_latitude = latitude
            first_longitude = longitude
            print("First loaction save!")
    else:
        print("No GPS Signal")

if __name__ == '__main__':
    rospy.init_node("get_gps", anonymous=10)

    rospy.Subscriber("bebop/states/ardrone3/GPSState/NumverOfSatelliteChanged", Ardrone3GPSStateNumberOfSatelliteChanged, cbSatelliteNumCheck)
    rospy.Subscriber("bebop/states/ardrone3/PilotingState/PositionChanged", Ardrone3PilotingStatePositionChanged, cbFirstLocationSave)
    
    rospy.spin()