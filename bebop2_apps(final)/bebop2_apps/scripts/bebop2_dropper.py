#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

from py_lib.GetChar import GetChar

from serial import serial_for_url
from io import TextIOWrapper, BufferedRWPair

srl = serial_for_url('/dev/ttyUSB1', 9600, timeout=1)
sio = TextIOWrapper(BufferedRWPair(srl, srl))

msg4drop="1\r\n"
 
str = """
---------------------------------------------------
 1:take off,  2:landing,  3:emergency,  'Q':to quit
---------------------------------------------------
        w                           i                      
   a    s    d                j     k     l
---------------------------------------------------
w/s : up  / down           i/k : foward / backword
a/d : ccw / cw             j/l : left   / righ
---------------------------------------------------
-/+ : decrease / increase linear  speed by 10%
,/. : decrease / increase angular speed by 10%
---------------------------------------------------
any other key is for stop(just hovering)
"""
e = "Communications Failed"
 
keys4move = {
#         x, y, z, th        x, y, z, th        x, y, z, th        x, y, z, th
    'w':( 0, 0, 1, 0), 'a':( 0, 0, 0, 1), 'i':( 1, 0, 0, 0), 'j':( 0, 1, 0, 0),
    's':( 0, 0,-1, 0), 'd':( 0, 0, 0,-1), 'k':(-1, 0, 0, 0), 'l':( 0,-1, 0, 0),
}

keys4speed = {
#   '-':(lin, ang), '+':(lin, ang), '<':(lin, ang), '>':(lin, ang)
    '-':(0.9, 1.0), '=':(1.1, 1.0), ',':(1.0, 0.9), ',':(1.0, 1.1),
}

def get_speed(lin, ang):
    return "current speed:  linear = %s(m/s), angular = %s(rad/s)" % (lin, ang)
    #      "current speed:  linear = 0.5(m/s), angular = 1.0(rad/s)"
 
if __name__=="__main__":
 
    pub0 = rospy.Publisher('bebop/cmd_vel', Twist, queue_size = 1)
    pub1 = rospy.Publisher('bebop/takeoff', Empty, queue_size = 1)
    pub2 = rospy.Publisher('bebop/land',    Empty, queue_size = 1)
    
    empty_msg = Empty()
    key_input = GetChar()
    
    rospy.init_node('bb2_teleop')
    
    lin_spd = rospy.get_param("~speed", 0.5)
    ang_spd = rospy.get_param("~turn",  1.0)
    
    count = 0
    x = 0;    y = 0;    z = 0;    th = 0;
    
    try:
        print(str)
        print(get_speed(lin_spd,ang_spd))
        
        while not rospy.is_shutdown():
        
            key = key_input.getch()
                
            if(   key == '1' ):  # take off
                pub1.publish(empty_msg)
                
            elif( key == '2' ):  # land
                pub2.publish(empty_msg)
                
            elif( key == '3' ):  # emergency stop
                pub3.publish(empty_msg)
                
            elif( key == '0' ):  # drop payload
                sio.write(unicode(msg4drop))
                sio.flush()
        
            elif( key in keys4move.keys() ):  # change value for Twist() message
                x  = keys4move[key][0]
                y  = keys4move[key][1]
                z  = keys4move[key][2]
                th = keys4move[key][3]
                
            elif( key in keys4speed.keys() ): # change value for linear & angular speed
                lin_spd = ang_spd * keys4speed[key][0]
                ang_spd = ang_spd * keys4speed[key][1]
 
                print(get_speed(lin_spd,ang_spd))
                
            else:  # any key except '1', '2', '3', 'Q', keys4move[], keys4speed[]
                x  = 0;    y  = 0;    z  = 0;    th = 0
                if (key == 'Q'):  # 'Q' for end program
                    break
                
            count = (count + 1) % 20  # for print usage every 20 times key input
                    
            if (count == 0):
                print(str)
 
            tw = Twist()  # declare Twist object name tw
            
            tw.linear.x  = x  * lin_spd
            tw.linear.y  = y  * lin_spd
            tw.linear.z  = z  * lin_spd
            
            tw.angular.x = 0
            tw.angular.y = 0
            tw.angular.z = th * ang_spd
            
            pub0.publish(tw)
            
        pub2.publish(empty_msg);  print("land")
        
    except rospy.ROSInterruptException:
        
        pub2.publish(empty_msg);  print("land")

