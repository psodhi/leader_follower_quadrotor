#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 UAVenture AG.
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA

import rospy
import thread
import threading
import time

from geometry_msgs.msg import PoseStamped, Quaternion, PoseWithCovarianceStamped
from math import *
from mavros.srv import CommandBool
from mavros.utils import *
from std_msgs.msg import Header
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from sp_pos.msg import utmData

utmXRef = 0.0
utmYRef = 0.0
utmXHome = 0.0
utmYHome = 0.0
flag = 0

class Setpoint:

    def __init__(self, pub, rospy):
        self.pub = pub
        self.rospy = rospy

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        try:
            thread.start_new_thread( self.navigate, () )
        except:
            print "Error: Unable to start thread"

        # TODO(simon): Clean this up.
        self.done = False
        self.done_evt = threading.Event()
        sub = rospy.Subscriber('/mavros/position/local', PoseStamped, self.reached)

    def navigate(self):
        rate = self.rospy.Rate(10) # 10hz
        
        msg = PoseStamped()
        msg.header = Header() 
        msg.header.frame_id = "base_footprint"
        msg.header.stamp = rospy.Time.now()

        while 1:
            msg.pose.position.x = self.x
            msg.pose.position.y = self.y
            msg.pose.position.z = self.z

            # For demo purposes we will lock yaw/heading to north.
            yaw_degrees = 0  # North
            yaw = radians(yaw_degrees)
            quaternion = quaternion_from_euler(0, 0, yaw)
            msg.pose.orientation = Quaternion(*quaternion)

            self.pub.publish(msg)

            rate.sleep()

    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z

        if wait:
            rate = rospy.Rate(5)
            while not self.done:
                rate.sleep()
        
        time.sleep(delay)


    def reached(self, topic):
            #print topic.pose.position.z, self.z, abs(topic.pose.position.z - self.z)
            if abs(topic.pose.position.x - self.x) < 2 and abs(topic.pose.position.y - self.y) < 2 and abs(topic.pose.position.z - self.z) < 1:
                self.done = True

            self.done_evt.set()

def utmRefCb(msg_utm_ref):
    global utmXRef, utmYRef, utmXHome, utmYHome, flag
    utmYRef = msg_utm_ref.utmNorthing
    utmXRef = msg_utm_ref.utmEasting
    if (flag == 0):
        utmXHome = msg_utm_ref.utmEasting
        utmYHome = msg_utm_ref.utmNorthing
        print "utmXHome" + str(utmXHome)
        print "utmYHome" + str(utmYHome)
        flag = 1
    
def utmCurrCb(msg_utm_curr):
    flag1 = 8
    #global utmXCurr, utmYCurr
    #global utmXHome, utmYHome, flag
    #if (flag == 8):
    #    utmXHome = msg_utm_curr.pose.pose.position.x
    #    utmYHome = msg_utm_curr.pose.pose.position.y
    #    print "utmXHome" + str(utmXHome)
    #    print "utmYHome" + str(utmYHome)
    #    flag = 1
    
def setpoint_demo():
    global utmXRef, utmYRef, utmXHome, utmYHome
    
    pub = rospy.Publisher('/mavros/setpoint/local_position', PoseStamped, queue_size=1)
    
    rospy.init_node('lpos_control', anonymous=True)
    rate = rospy.Rate(10) 

    setpoint = Setpoint(pub,rospy)

    sub_utm_ref = rospy.Subscriber('/gps/utm', utmData, utmRefCb)
    sub_utm_curr = rospy.Subscriber('/mavros/global_position/local', PoseWithCovarianceStamped, utmCurrCb)
    
    distLF = 0.0
    height = 5.0
    delay = 0.0
    wait = False

    x_test = 8
    y_test = 6
    
#    print "Climb"
#    setpoint.set(0.0, 0.0, 3.0, 0)
#    setpoint.set(0.0, 0.0, height, 5)

    print "Climb"
    setpoint.set(0.0, 0.0, 5.0, 5)   # Climb to the starting height first
    #print "Fly to the right"
    #setpoint.set(10.0, 4.0, 5.0, 5)
    #print "Fly to the left"
    #setpoint.set(0.0, 0.0, 5.0, 5)
    
    while not rospy.is_shutdown():
        #setpoint.set(x_test, y_test, height, delay, wait)
        #x_test = -x_test
        #y_test = -y_test
        setpoint.set(utmXRef-utmXHome-distLF, utmYRef-utmYHome-distLF, height, delay, wait)
        #print "utmXRef" + str(utmXRef)
        #print "utmYRef" + str(utmYRef)
        print "XLocal" + str(utmXRef-utmXHome)
        print "YLocal" + str(utmYRef-utmYHome)
        rate.sleep()
    
    # print "Climb"
    # setpoint.set(0.0, 0.0, 3.0, 0)
    # setpoint.set(0.0, 0.0, 10.0, 5)

    # print "Sink"
    # setpoint.set(0.0, 0.0, 8.0, 5)

    # print "Fly to the right"
    # setpoint.set(10.0, 4.0, 8.0, 5)

    # print "Fly to the left"
    # setpoint.set(0.0, 0.0, 8.0, 5)

    # offset_x = 0.0
    # offset_y = 0.0
    # offset_z = 10.0
    # sides = 360
    # radius = 20

    # print "Fly in a circle"
    # setpoint.set(0.0, 0.0, 10.0, 3)   # Climb to the starting height first
    # i = 0
    # while not rospy.is_shutdown()
:    #     x = radius * cos(i*2*pi/sides) + offset_x   
    #     y = radius * sin(i*2*pi/sides) + offset_y
    #     z = offset_z

    #     wait = False
    #     delay = 0
    #     if (i == 0 or i == sides):
    #         # Let it reach the setpoint.
    #         wait = True
    #         delay = 5

    #     setpoint.set(x, y, z, delay, wait)

    #     i = i + 1 
    #     rate.sleep()

    #     if (i > sides):
    #         print "Fly home"
    #         setpoint.set(0.0, 0.0, 10.0, 5)
    #         break

    # # Simulate a slow landing.
    # setpoint.set(0.0, 0.0,  8.0, 5)
    # setpoint.set(0.0, 0.0,  3.0, 5)
    # setpoint.set(0.0, 0.0,  2.0, 2)
    # setpoint.set(0.0, 0.0,  1.0, 2)
    # setpoint.set(0.0, 0.0,  0.0, 2)
    # setpoint.set(0.0, 0.0, -0.2, 2)

    # print "Bye!"


if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass