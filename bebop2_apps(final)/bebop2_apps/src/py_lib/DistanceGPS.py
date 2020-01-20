#! /usr/bin/env python
 
from math import pi, sqrt, sin, cos, tan, acos, asin, atan
  
# class for getting distance & azimuth between P1 & P2
class DistanceGPS:

    def __init__(self):
        self.c15 = 6378137.000000000
        self.c16 = 6356752.314140910
        self.c17 =       0.0033528107
      
    def get_distance(self, P1_latitude, P1_longitude, P2_latitude, P2_longitude):
        
        # convert from degree to radian
        e10 = P1_latitude  * pi / 180.
        e11 = P1_longitude * pi / 180.
        e12 = P2_latitude  * pi / 180.
        e13 = P2_longitude * pi / 180.
        
        # GRS80
        f15 = self.c17 + self.c17 * self.c17
        f16 = f15 / 2.
        f17 = self.c17 * self.c17 /  2.
        f18 = self.c17 * self.c17 /  8.
        f19 = self.c17 * self.c17 / 16.
        
        c18 = e13 - e11
        c20 = (1. - self.c17) * tan(e10)
        c21 = atan(c20)
        c22 = sin(c21)
        c23 = cos(c21)
        c24 = (1. - self.c17) * tan(e12)
        c25 = atan(c24)
        c26 = sin(c25)
        c27 = cos(c25)
        c29 = c18
        c31 = (c27 * sin(c29) * c27 * sin(c29)) + (c23 * c26 - c22 * c27 * cos(c29)) * (c23 * c26 - c22 * c27 * cos(c29))
        c33 = (c22 * c26) + (c23 * c27 * cos(c29))
        c35 = sqrt(c31) / c33
        c36 = atan(c35)
        c38 = 0.0
        c40 = 0.0
        c41 = cos(asin(c38)) * cos(asin(c38)) * (self.c15 * self.c15 - self.c16 * self.c16) / (self.c16 * self.c16)
        c43 = 1 + c41 / 16384 * (4096 + c41 * (-768 + c41 * (320 - 175 * c41)))
        c45 = c41 / 1024 * (256 + c41 * (-128 + c41 * (74 - 47 * c41)))
        c47 = c45 * sqrt(c31) * (c40 + c45 / 4 * (c33 * (-1 + 2 * c40 * c40) - c45 / 6 * c40 * (-3 + 4 * c31) * (-3 + 4 * c40 * c40)))
        c50 = self.c17 / 16 * cos(asin(c38)) * cos(asin(c38)) * (4 + self.c17 * (4 - 3 * cos(asin(c38)) * cos(asin(c38))))
        c52 = c18 + (1 - c50) * self.c17 * c38 * (acos(c33) + c50 * sin(acos(c33)) * (c40 + c50 * c33 * (-1 + 2 * c40 * c40)))
        c54 = self.c16 * c43 * (atan(c35) - c47)
    
        if ((P1_latitude == P2_latitude) and (P1_longitude == P2_longitude)):
            return 0
        
        if (c31 == 0):
            c38 = 0.0
            
        else:
            c38 = c23 * c27 * sin(c29) / sqrt(c31)
        
        if ((cos(asin(c38)) * cos(asin(c38))) == 0):
            c40 = 0.0
            
        else:
            c40 = c33 - 2 * c22 * c26 / (cos(asin(c38)) * cos(asin(c38)))
        
        return c54
        
  
    def get_bearing(self, P1_latitude, P1_longitude, P2_latitude, P2_longitude):
    
        Cur_Lat_radian  = P1_latitude  * (pi / 180)
        Cur_Lon_radian  = P1_longitude * (pi / 180)
        
        Dest_Lat_radian = P2_latitude  * (pi / 180)
        Dest_Lon_radian = P2_longitude * (pi / 180)
        
        # radian distance
        radian_distance = 0;
        radian_distance = acos(sin(Cur_Lat_radian) * sin(Dest_Lat_radian) + cos(Cur_Lat_radian) * cos(Dest_Lat_radian) * cos(Cur_Lon_radian - Dest_Lon_radian))
        
        radian_bearing = acos((sin(Dest_Lat_radian) - sin(Cur_Lat_radian) * cos(radian_distance)) / (cos(Cur_Lat_radian) * sin(radian_distance)))
        true_bearing = 0
        
        if (sin(Dest_Lon_radian - Cur_Lon_radian) < 0):
            true_bearing = radian_bearing * (180 / pi)
            true_bearing = 360 - true_bearing
        else:
            true_bearing = radian_bearing * (180 / pi)
            
        return true_bearing



