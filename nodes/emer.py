#!/usr/bin/env python

# This code is build to use triangulation approach for geo-location a identified target in ROS
# Written by Ruoyu Tan
# Air Vehicle Intelligence and Autonomy Lab(Director: Dr. Jack Langelaan),
# The Pennsylvania State University, USA.
# Last updated: Sept 2014
# Please acknowledge in any academic papers which may utilise this code

import os
import sys
import struct
import time

import roslib; roslib.load_manifest('roscopter')
import rospy
from std_msgs.msg import String, Header
from std_srvs.srv import *
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus

import roscopter.msg

pub = rospy.Publisher('send_rc', roscopter.msg.RC)
rospy.init_node('command')
t=0
while not rospy.is_shutdown():
   if t<800:
	   #publish pitch/roll/thrust/yaw/flight mode/Aux 1/Aux 2/Aux 3 command
	   #range are all from 1000---2000; for example 1500 means RC stick in the middle position
       pub.publish([1544,1499,1170,1522,0,0,0,0])
   else:
       pub.publish([1544,1499,1020,1522,0,0,0,0])
   t=t+1
   print(t)
   rospy.sleep(0.01)

#pitch roll yaw 63

#throttle 9-127

#0 63 127
