#!/usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import NavSatFix
from sp_pos.msg import utmData, latlonData
import math
import time

# Home GPS Locattions
# latitude = 13.0271196;
# longitude = 77.5631332;

M_PI = 3.14159265358979323846
WGS84_A = 6378137.0 # const double
WGS84_ECCSQ = 0.00669437999013 # const double

def utmLetterDesignator(latitude): # double latitude
  # This routine determines the correct UTM letter designator for the given latitude
  # returns 'Z' if latitude is outside the UTM limits of 84N to 80S
  # Written by Chuck Gantz- chuck.gantz@globalstar.com

  if ((latitude <= 84.0) and (latitude >= 72.0)):
    letterDesignator = 'X'
  elif ((latitude < 72.0) and (latitude >= 64.0)):
    letterDesignator = 'W'
  elif ((latitude < 64.0) and (latitude >= 56.0)):
    letterDesignator = 'V'
  elif ((latitude < 56.0) and (latitude >= 48.0)):
    letterDesignator = 'U'
  elif ((latitude < 48.0) and (latitude >= 40.0)):
    letterDesignator = 'T'
  elif ((latitude < 40.0) and (latitude >= 32.0)):
    letterDesignator = 'S'
  elif ((latitude < 32.0) and (latitude >= 24.0)):
    letterDesignator = 'R'
  elif ((latitude < 24.0) and (latitude >= 16.0)):
    letterDesignator = 'Q'
  elif ((latitude < 16.0) and (latitude >= 8.0)):
    letterDesignator = 'P'
  elif (( latitude < 8.0) and (latitude >= 0.0)):
    letterDesignator = 'N'
  elif (( latitude < 0.0) and (latitude >= -8.0)):
    letterDesignator = 'M'
  elif ((latitude < -8.0) and (latitude >= -16.0)):
    letterDesignator = 'L'
  elif ((latitude < -16.0) and (latitude >= -24.0)):
    letterDesignator = 'K'
  elif ((latitude < -24.0) and (latitude >= -32.0)):
    letterDesignator = 'J'
  elif ((latitude < -32.0) and (latitude >= -40.0)):
    letterDesignator = 'H'
  elif ((latitude < -40.0) and (latitude >= -48.0)):
    letterDesignator = 'G'
  elif ((latitude < -48.0) and (latitude >= -56.0)):
    letterDesignator = 'F'
  elif ((latitude < -56.0) and (latitude >= -64.0)):
    letterDesignator = 'E'
  elif ((latitude < -64.0) and (latitude >= -72.0)):
    letterDesignator = 'D'
  elif ((latitude < -72.0) and (latitude >= -80.0)):
    letterDesignator = 'C'
  else:
    letterDesignator = 'Z' # This is here as an error flag to show that the Latitude is outside the UTM limits

  return letterDesignator

def lltoutm(latitude, longitude):
  # def lltoutm(double latitude, double longitude, double& utmNorthing, double& utmEasting, std::string& utmZone):
  # converts lat/long to UTM coords.  Equations from USGS Bulletin 1532
  # East Longitudes are positive, West longitudes are negative.
  # North latitudes are positive, South latitudes are negative
  # Lat and Long are in decimal degrees
  # Written by Chuck Gantz- chuck.gantz@globalstar.com

  k0 = 0.9996 # double

  LatRad = latitude * M_PI / 180.0 # double
  LongRad = longitude * M_PI / 180.0 # double

  ZoneNumber = int(((longitude + 180.0) / 6.0) + 1)

  if (latitude >= 56.0) and (latitude < 64.0) and (longitude >= 3.0) and (longitude < 12.0):
      ZoneNumber = 32

  # Special zones for Svalbard
  if (latitude >= 72.0) and (latitude < 84.0):
    if (longitude >= 0.0)  and (longitude <  9.0):
      ZoneNumber = 31
    elif (longitude >= 9.0)  and (longitude < 21.0):
      ZoneNumber = 33
    elif (longitude >= 21.0) and (longitude < 33.0):
      ZoneNumber = 35
    elif (longitude >= 33.0) and (longitude < 42.0):
      ZoneNumber = 37

  #LongOrigin = static_cast<double>((ZoneNumber - 1) * 6 - 180 + 3);  # +3 puts origin in middle of zone
  LongOrigin = float(((ZoneNumber - 1) * 6 - 180 + 3))  # +3 puts origin in middle of zone
  LongOriginRad = LongOrigin * M_PI / 180.0

  # compute the UTM Zone from the latitude and longitude
  utmZone = str(ZoneNumber) + utmLetterDesignator(latitude)

  eccPrimeSquared = WGS84_ECCSQ / (1.0 - WGS84_ECCSQ)

  N = WGS84_A / math.sqrt(1.0 - WGS84_ECCSQ * math.sin(LatRad) * math.sin(LatRad))
  T = math.tan(LatRad) * math.tan(LatRad)
  C = eccPrimeSquared * math.cos(LatRad) * math.cos(LatRad)
  A = math.cos(LatRad) * (LongRad - LongOriginRad)

  M = WGS84_A * ((1.0 - WGS84_ECCSQ / 4.0
                  - 3.0 * WGS84_ECCSQ * WGS84_ECCSQ / 64.0
                  - 5.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 256.0)
                 * LatRad
                 - (3.0 * WGS84_ECCSQ / 8.0
                    + 3.0 * WGS84_ECCSQ * WGS84_ECCSQ / 32.0
                    + 45.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 1024.0)
                 * math.sin(2.0 * LatRad)
                 + (15.0 * WGS84_ECCSQ * WGS84_ECCSQ / 256.0
                    + 45.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 1024.0)
                 * math.sin(4.0 * LatRad)
                 - (35.0 * WGS84_ECCSQ * WGS84_ECCSQ * WGS84_ECCSQ / 3072.0)
                 * math.sin(6.0 * LatRad))

  utmEasting = k0 * N * (A + (1.0 - T + C) * A * A * A / 6.0
                         + (5.0 - 18.0 * T + T * T + 72.0 * C
                            - 58.0 * eccPrimeSquared)
                         * A * A * A * A * A / 120.0) + 500000.0

  utmNorthing = k0 * (M + N * math.tan(LatRad) *
                      (A * A / 2.0 +
                       (5.0 - T + 9.0 * C + 4.0 * C * C) * A * A * A * A / 24.0
                       + (61.0 - 58.0 * T + T * T + 600.0 * C
                          - 330.0 * eccPrimeSquared)
                       * A * A * A * A * A * A / 720.0))
  if (latitude < 0.0):
    utmNorthing += 10000000.0 # 10000000 meter offset for southern hemisphere

  return (utmNorthing,utmEasting,utmZone)

def callback0(msg_latlon):
  latitude = msg_latlon.latitude
  longitude = msg_latlon.longitude
  
  # AE-147!
  #latitude = 13.0271196
  #longitude = 77.5631332
  
  utmFuncReturn = lltoutm(latitude, longitude)
  
  msg_utm = utmData()
  msg_utm.utmNorthing = utmFuncReturn[0] 
  msg_utm.utmEasting = utmFuncReturn[1]
  msg_utm.utmZone = utmFuncReturn[2]

  pub_utm.publish(msg_utm)

  #print "UTM Northing: " + str(msg_utm.utmNorthing) + " UTM Easting: " + str(msg_utm.utmNEasting) + " UTM Zone: " + str(msg_utm.utmZone)
 
if __name__ == '__main__':
  
  try:

    pub_utm = rospy.Publisher('/gps/utm', utmData, queue_size=1)
    #rospy.Subscriber('/fix', NavSatFix, callback0)
    rospy.Subscriber('/gps/latlon', latlonData, callback0)
    rospy.init_node('gps_utm_node', anonymous=True)
    
    r = rospy.Rate(10) # 10hz
    
    while not rospy.is_shutdown():  
      r.sleep()
  except rospy.ROSInterruptException: 
    pass